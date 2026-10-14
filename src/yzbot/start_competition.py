#!/usr/bin/env python3
"""Managed one-command launcher for the AIC ROS 2 competition system."""

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time

import yaml
from datetime import datetime
from pathlib import Path


ROS_SETUP = Path("/opt/ros/humble/setup.bash")
WORKSPACE_SRC = Path(__file__).resolve().parent
sys.path.insert(0, str(WORKSPACE_SRC / "aic_task"))
from aic_task.scene_manager import load_scene, validate_scene
from aic_task.navigation.object_docking.profile import fixed_world2_parameters
WORKSPACE = WORKSPACE_SRC.parent.parent
WORKSPACE_SETUP = WORKSPACE / "install/setup.bash"
FOXGLOVE_DIR = WORKSPACE_SRC / "foxglove"
FOXGLOVE_LAUNCHER = FOXGLOVE_DIR / "start_offline_foxglove.sh"
FOXGLOVE_ARCHIVE = FOXGLOVE_DIR / "offline_studio_1.87.0_web.tar.gz"
LOG_BASE = Path.home() / "aic_competition_delivery/logs"
SCENES_ROOT = WORKSPACE_SRC / "aic_task/config/scenes"
INITIAL_POSE = (0.0, 0.0, 0.0)


def _env_int(name, default):
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


# 2026-10-14：一次 20 轮测试把根分区写满，机器随之宕机（syslog 13:54:52 ENOSPC）。
# 子进程日志此前直接继承 launcher 打开的文件描述符，父进程无法限长——改名或截断都改不动
# 子进程手里的 fd 偏移，所以任何刷屏节点都能一路写满磁盘。现在子进程统一走 PIPE，
# 由父进程限额落盘；再加一个磁盘看门狗，空间不足时先报出写入最快的进程，再优雅停机。
CHILD_LOG_MAX_BYTES = max(0, _env_int("AIC_LOG_MAX_MB", 32)) * 1024 * 1024
MIN_FREE_BYTES = max(0, _env_int("AIC_MIN_FREE_GB", 15)) * 1024 ** 3
DISK_GUARD_INTERVAL = 10.0


class _ChildLogDrain(threading.Thread):
    """父进程侧限额落盘：读子进程 PIPE，超出上限的部分丢弃并计数。

    不能改成给子进程一个文件再由父进程轮转：子进程持有自己的 fd，父进程 rename/truncate
    既不会把它的写偏移拉回 0，也不会让它换文件，日志照样无限增长。
    """

    def __init__(self, name, path, max_bytes, stream):
        super().__init__(name=f"log-drain-{name}", daemon=True)
        self.path = Path(path)
        self.max_bytes = max_bytes
        self.stream = stream
        self.written = 0
        self.dropped = 0
        self.handle = open(self.path, "wb", buffering=0)
        self._stop = threading.Event()

    def run(self):
        fd = self.stream.fileno()
        try:
            while not self._stop.is_set():
                # os.read 只要管道里有数据就返回。原来的 BufferedReader.read(n) 会一直
                # 阻塞到凑满 n 字节或 EOF：结果安静节点的日志恒为 0（gazebo/moveit/
                # task_parser 实测 0 字节），刷屏节点卡在 64KB 整倍数，而且节点自己会
                # 因为管道写满而阻塞。
                chunk = os.read(fd, 65536)
                if not chunk:
                    break
                if self.max_bytes and self.written >= self.max_bytes:
                    self.dropped += len(chunk)
                    continue
                room = chunk if not self.max_bytes else chunk[: self.max_bytes - self.written]
                self.handle.write(room)
                self.written += len(room)
                self.dropped += len(chunk) - len(room)
        except (OSError, ValueError):
            pass
        finally:
            try:
                self.handle.flush()
            except (OSError, ValueError):
                pass

    def summary(self):
        text = f"{self.written / 1e6:.1f}MB kept"
        if self.dropped:
            text += f", {self.dropped / 1e6:.1f}MB dropped (cap {self.max_bytes / 1e6:.0f}MB)"
        return text

    def close(self):
        self._stop.set()
        for target in (self.stream, self.handle):
            try:
                target.close()
            except (OSError, ValueError):
                pass


class _DiskGuard(threading.Thread):
    """采样剩余空间与各进程写入速率；空间见底时先报告写入者再停机。"""

    def __init__(self, launcher):
        super().__init__(name="disk-guard", daemon=True)
        self.launcher = launcher
        self._stop = threading.Event()
        self._previous = {}

    def _sample(self):
        now = time.monotonic()
        rates = {}
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            try:
                text = (entry / "io").read_text()
                written = int(re.search(r"write_bytes:\s*(\d+)", text).group(1))
                name = (entry / "comm").read_text().strip()
            except (OSError, AttributeError, ValueError):
                continue
            pid = int(entry.name)
            previous = self._previous.get(pid)
            if previous is not None:
                elapsed = max(1e-6, now - previous[1])
                rates[pid] = (name, max(0.0, (written - previous[0]) / elapsed))
            self._previous[pid] = (written, now)
        return rates

    @staticmethod
    def _top(rates, count=3):
        ranked = sorted(rates.items(), key=lambda item: item[1][1], reverse=True)[:count]
        return ", ".join(f"{name}(pid {pid}) {rate / 1e6:.1f}MB/s" for pid, (name, rate) in ranked)

    def run(self):
        while not self._stop.wait(DISK_GUARD_INTERVAL):
            try:
                free = shutil.disk_usage(self.launcher.run_dir).free
            except OSError:
                continue
            rates = self._sample()
            if free < MIN_FREE_BYTES:
                self.launcher.log(
                    f"FATAL: free disk {free / 1e9:.1f}GB below AIC_MIN_FREE_GB="
                    f"{MIN_FREE_BYTES / 1e9:.0f}; fastest writers: {self._top(rates)}"
                )
                for name, drain in list(self.launcher.log_handles.items()):
                    if getattr(drain, "dropped", 0):
                        self.launcher.log(f"[disk] {name} log {drain.summary()}")
                self.launcher._shutdown()
                os._exit(3)
            self.launcher.log(f"[disk] free={free / 1e9:.1f}GB fastest writers: {self._top(rates)}")

    def close(self):
        self._stop.set()



def log_base_dir():
    """解析本次运行的日志根目录。

    默认仍是历史路径 `~/aic_competition_delivery/logs`，保证未设环境变量时行为不变。
    该目录可能已被清理而不存在；正式启动前必须把日志落盘到本次 Work 记录目录，
    否则 `_create_run_dir` 会因父目录缺失直接抛异常。用 `AIC_LOG_BASE` 指定绝对路径。
    """
    override = os.environ.get("AIC_LOG_BASE", "").strip()
    if not override:
        return LOG_BASE
    candidate = Path(override).expanduser()
    if not candidate.is_absolute():
        raise ValueError(f"AIC_LOG_BASE must be an absolute path, got {override!r}")
    return candidate

# --- 运行期 lifecycle 保护（见 aic_task/lifecycle_watchdog.py 与 CR-3）---
# Nav2 lifecycle_manager 的 bond 心跳默认 4 s。本 VM 满载（load average > 10）时心跳可能被漏掉，
# 于是 `bond_map_[node]->isBroken()` 为真 → 判为 CRITICAL FAILURE，把**它管理的全部节点**
# 一起 deactivate（Nav2 设计行为："Shutting down related nodes"），且**不会自动恢复**。
# velocity_smoother 一旦 inactive，/cmd_vel_nav 上的命令全部到不了底盘 →
# 机器人在 Nav2 与编排器控制下完全不能动，而其它话题看起来都正常。
#
# 注意：**不能用运行时 `ros2 param set bond_timeout` 来放宽这个窗口**。
# Nav2 Humble 的 lifecycle_manager 在构造函数里 `declare_parameter("bond_timeout", 4.0)`
# 后立刻 `get_parameter` 并缓存进成员 `bond_timeout_`，之后只读成员。
# 实测：参数读回 30.0，但 CRITICAL FAILURE 报文仍打印 4000 ms。
# 因此本启动器**不再尝试改它**，改用运行期看门狗兜底（掉了自动接回并上报）。

# --- Foxglove 窗口选择（与 foxglove/foxglove_window_selection.sh 行为必须一致）---
# 契约由 foxglove/test_foxglove_window_contract.py 守住：两边对同一批输入必须给出
# 相同的窗口集合与端口，否则启动器的就绪门会和实际起的 HTTP 服务对不上。
FOXGLOVE_WINDOW_DEFAULTS = {"map": 8088, "detection": 8089, "dashboard": 8090}
FOXGLOVE_DEFAULT_WINDOW = "dashboard"

# 可视化辅助进程：退出只降级、不判比赛失败。见 _wait_shutdown 的说明。
FOXGLOVE_OPTIONAL_PROCESSES = ("foxglove_runtime", "foxglove_studio")


def workspace_build_base(workspace):
    """ROSIDL的file(STRINGS)会拆断中文路径，输出缓存使用ASCII目录。"""
    resolved = (Path(workspace) / "build").resolve()
    if str(resolved).isascii():
        return resolved
    identity = hashlib.sha256(str(Path(workspace).resolve()).encode("utf-8")).hexdigest()[:12]
    return Path.home() / ".cache" / "aic_competition_build" / identity / "build"


def rosbridge_port():
    """面板、就绪检查和运行记录统一使用的本机端口。"""
    try:
        port = int(os.environ.get("ROSBRIDGE_PORT", "9091"))
    except (TypeError, ValueError) as exc:
        raise ValueError("ROSBRIDGE_PORT must be an integer in 1..65535") from exc
    if not 1 <= port <= 65535:
        raise ValueError("ROSBRIDGE_PORT must be an integer in 1..65535")
    return port


def foxglove_selected_windows(value=None):
    """归一化 FOXGLOVE_WINDOWS，返回去重后的窗口元组。

    空/未设置 -> ("dashboard",)（照 test13 性能记录 §5：默认只开仪表盘）
    all/ALL   -> 三个窗口都开
    其余       -> 逗号或空格分隔，未知项抛 ValueError
    """
    if value is None:
        value = os.environ.get("FOXGLOVE_WINDOWS", "")
    text = str(value or "").strip()
    if not text:
        return (FOXGLOVE_DEFAULT_WINDOW,)
    if text.lower() == "all":
        return ("map", "detection", "dashboard")
    parts = [part for part in text.replace(",", " ").split() if part]
    if not parts:
        return (FOXGLOVE_DEFAULT_WINDOW,)
    unknown = [part for part in parts if part not in FOXGLOVE_WINDOW_DEFAULTS]
    if unknown:
        raise ValueError(
            "Unknown FOXGLOVE_WINDOWS entry: %s (use %s|all, comma separated)"
            % (
                ", ".join(unknown),
                "|".join(sorted(FOXGLOVE_WINDOW_DEFAULTS)),
            )
        )
    unique = []
    for part in parts:
        if part not in unique:
            unique.append(part)
    return tuple(unique)


def foxglove_window_ports(windows=None):
    """返回所选窗口的端口列表（含 FOXGLOVE_<WINDOW>_PORT 环境变量覆盖）。"""
    if windows is None:
        windows = foxglove_selected_windows()
    ports = []
    for window in windows:
        name = "FOXGLOVE_%s_PORT" % window.upper()
        default = FOXGLOVE_WINDOW_DEFAULTS[window]
        try:
            ports.append(int(os.environ.get(name, default)))
        except (TypeError, ValueError):
            ports.append(default)
    return ports

NAV2_LIFECYCLE_NODES = (
    "map_server",
    "amcl",
    "controller_server",
    "planner_server",
    "bt_navigator",
    "behavior_server",
    "collision_monitor",
)


def ros_file_parameter(path, workspace=None):
    """Keep serialized ROS file values ASCII and scoped to the runtime workspace."""
    root = Path(workspace if workspace is not None else WORKSPACE).resolve()
    try:
        relative = Path(path).resolve().relative_to(root).as_posix()
    except ValueError as exc:
        raise ValueError(f"ROS file parameter must remain inside workspace: {path}") from exc
    if not relative.isascii():
        raise ValueError(f"ROS file parameter must have an ASCII relative path: {relative}")
    return relative


class CompetitionLauncher:
    def __init__(self, args):
        self.args = args
        self.mode = args.mode
        # --reset must still work when active.yaml or a scene file is broken.
        self.scene = None if getattr(args, "reset", False) else load_scene(
            args.world, SCENES_ROOT, WORKSPACE
        )
        self.world = self.scene.scene_id if self.scene is not None else "reset"
        self.points_file = self.scene.points_file if self.scene is not None else None
        self.path_matrix = self.scene.matrix_file if self.scene is not None else None
        gazebo_prefix = self.scene.service_prefix if self.scene is not None else ""
        self.get_entity_service = f"{gazebo_prefix}/get_entity_state"
        self.set_entity_service = f"{gazebo_prefix}/set_entity_state"
        self.display = args.display
        self.ros_domain_id = args.ros_domain_id
        self.initial_pose = (
            float(args.initial_pose_x),
            float(args.initial_pose_y),
            float(args.initial_pose_yaw),
        )
        # 未显式指定初始位置（= 默认 INITIAL_POSE）时，按当前 world 从 points 文件取
        # robot_initial_pose，使 v1 / world2 两套场景各自的起点自动生效
        # （world2 的物块与区域都在房间下半部，起点必须跟着走，否则车停在 12 m 外）。
        if self.scene is not None and self.initial_pose == INITIAL_POSE:
            from_points = self._initial_pose_from_points()
            if from_points is not None:
                self.initial_pose = from_points
        if self.scene is None:
            self.dynamic_obstacle_names = ()
        elif args.dynamic_obstacle_names == "all":
            self.dynamic_obstacle_names = self.scene.dynamic_obstacles
        else:
            self.dynamic_obstacle_names = tuple(
                dict.fromkeys(
                    name.strip()
                    for name in args.dynamic_obstacle_names.split(",")
                    if name.strip()
                )
            )
        self.processes = {}
        self.log_handles = {}
        self._disk_guard = None
        self.stage_status = {}
        self.start_time = datetime.now()
        self.run_dir = None
        self._stopping = False
        self._shutdown_requested = False
        self._shutdown_signal = None
        self._heartbeat_stop = threading.Event()
        self._heartbeat_thread = None

    def _initial_pose_from_points(self):
        """按当前 world 的 points 文件取 robot_initial_pose；缺失或异常时返回 None。

        用于让 v1 / world2 两套场景各自的起点自动生效（CLI 显式指定时不会走到这里）。
        """
        try:
            with open(self.points_file, encoding="utf-8") as fh:
                points = yaml.safe_load(fh) or {}
        except FileNotFoundError:
            return None
        except Exception as exc:  # 配置读坏了不应阻塞启动，回退 CLI 值
            print(f"[initial_pose] 读取 {self.points_file} 失败: {exc}")
            return None
        initial = points.get("robot_initial_pose") or {}
        try:
            pose = (
                float(initial.get("x", 0.0)),
                float(initial.get("y", 0.0)),
                float(initial.get("yaw", 0.0)),
            )
        except (TypeError, ValueError):
            return None
        print(f"[initial_pose] world={self.world} 取自 points: {pose}")
        return pose

    def log(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        print(f"[{timestamp}] {message}", flush=True)

    def run(self):
        self.log("AIC competition launcher")
        # 先校验 Foxglove 窗口选择：非法值要在起任何进程之前就报错，
        # 不然会白等 45 秒就绪门再 FATAL。
        if not self.args.no_foxglove:
            self.foxglove_windows = foxglove_selected_windows()
            self.foxglove_http_ports = foxglove_window_ports(self.foxglove_windows)
        else:
            self.foxglove_windows = ()
            self.foxglove_http_ports = []
        self._degraded_subsystems = {}
        self.log(
            f"mode={self.mode} world={self.world} display={self.display} "
            f"ros_domain_id={self.ros_domain_id} "
            f"time_limit_sec={self.args.time_limit_sec or 'default'} "
            f"dynamic_obstacles={'off' if self.args.no_dynamic_obstacles else 'on'} "
            f"dynamic_obstacle_names={list(self.dynamic_obstacle_names)} "
            f"task_parser={'off' if self.args.no_task_parser else 'on'} "
            f"development_cube_start_index={self.args.development_cube_start_index} "
            f"initial_pose={self.initial_pose} "
            f"foxglove_windows={','.join(self.foxglove_windows) or 'off'} "
            f"foxglove_nice={os.environ.get('FOXGLOVE_NICE', 'default')} "
            f"lifecycle_watchdog={'off' if self.args.no_lifecycle_watchdog else 'on'}"
        )
        self._setup_environment()
        if self.world == "world2":
            self.log("World2 docking fixed in code: " + json.dumps(fixed_world2_parameters(), sort_keys=True))

        if self.args.reset:
            self._cleanup_residual()
            self.log("Residual competition processes cleaned up.")
            return

        if self.args.setup:
            self._build_workspace()
            return

        self._verify_prerequisites()

        if self.args.dry_run:
            self.log("Dry run passed prerequisite checks.")
            return

        self._create_run_dir()
        self._cleanup_residual()
        self._start_heartbeat()
        self._disk_guard = _DiskGuard(self)
        self._disk_guard.start()

        try:
            self._launch_gazebo()
            self._mark_stage("gazebo", self._wait_gazebo_ready())

            self._launch_moveit()
            self._mark_stage("moveit_graph", self._wait_moveit_ready())
            self._mark_stage("moveit_probe", self._run_moveit_probe())
            self._mark_stage("robot_initial_pose", self._reset_robot_initial_pose())

            if self.dynamic_obstacle_names and not self.args.no_dynamic_obstacles:
                self._launch_dynamic_obstacles()
                self._mark_stage(
                    "dynamic_obstacles", self._wait_dynamic_obstacles_configured()
                )
            elif self.dynamic_obstacle_names:
                self._mark_stage("dynamic_obstacles", "disabled_for_static_acceptance")

            self._launch_nav2()
            self._mark_stage("nav2_graph", self._wait_nav2_ready())
            self._set_initial_pose()
            self._mark_stage("localization", self._wait_localization_ready())
            self._mark_stage("nav2_active", self._wait_navigation_active())
            self._mark_stage("tf_integrity", self._check_tf_integrity())

            # 运行期保护：看门狗兜底（bond_timeout 在 Nav2 Humble 里无法在运行期修改，
            # 详见文件顶部注释）。轮询 1 s，把"掉线→接回"的窗口压到最小。
            self._launch_lifecycle_watchdog()

            self._launch_perception()
            self._mark_stage("perception", self._wait_perception_ready())

            self._launch_task_system()
            self._mark_stage("task_system", self._wait_task_ready())

            if not self.args.no_foxglove:
                self._launch_foxglove()
                self._mark_stage("foxglove_bridge", self._wait_foxglove_ready())

            if self.dynamic_obstacle_names and not self.args.no_dynamic_obstacles:
                self._enable_dynamic_obstacles()
                self._mark_stage('dynamic_obstacle_motion', self._wait_dynamic_obstacles_ready())
            self._write_manifest()
            self.log("SYSTEM READY: generate question, confirm mapping and parsed plan, then start.")
            self._wait_shutdown()
            self._shutdown()
        except Exception:
            self._write_manifest()
            self._shutdown()
            raise

    def _setup_environment(self):
        os.environ["ROSBRIDGE_PORT"] = str(rosbridge_port())
        os.environ["DISPLAY"] = self.display
        os.environ["ROS_DOMAIN_ID"] = str(self.ros_domain_id)
        # Keep all Gazebo/Nav2/task processes on the same DDS discovery scope.
        # The VM's competition chain needs non-localhost discovery for reliable
        # service and topic visibility across launch boundaries.
        os.environ["ROS_LOCALHOST_ONLY"] = os.environ.get("ROS_LOCALHOST_ONLY", "0")
        # 2026-09-27：交付包原样是 "0"（关闭共享内存），没有任何注释说明原因。
        # 实测症状：栈跑 20~30 min 后 DDS 开始堵 —— 激光扫描消息延迟 6 分钟（362 s）
        # 才到达 controller_server，触发 "timestamp earlier than all data in
        # transform cache"，随后 Nav2 节点连锁卡死（amcl lifecycle 超时 /
        # planner_server SIGABRT / controller_server 被 lifecycle_manager 判 DOWN）。
        # 打开 SHM 让 FastRTPS 走共享内存而不是 TCP/UDP 回环，绕过网络栈堵塞。
        # 回退：改回 "0"。
        os.environ["RMW_FASTRTPS_USE_SHM"] = "1"
        os.environ["PYTHONUNBUFFERED"] = "1"

        source = f"source {ROS_SETUP}"
        if WORKSPACE_SETUP.exists():
            source += f" && source {WORKSPACE_SETUP}"
        result = subprocess.run(
            ["bash", "-lc", f"{source} && env"],
            capture_output=True,
            text=True,
            timeout=15,
            check=True,
        )
        allowed = {
            "AMENT_PREFIX_PATH",
            "CMAKE_PREFIX_PATH",
            "GAZEBO_MODEL_PATH",
            "GAZEBO_PLUGIN_PATH",
            "GAZEBO_RESOURCE_PATH",
            "LD_LIBRARY_PATH",
            "PATH",
            "PYTHONPATH",
            "RMW_IMPLEMENTATION",
            "ROS_DISTRO",
            "ROS_VERSION",
        }
        for line in result.stdout.splitlines():
            key, separator, value = line.partition("=")
            if separator and key in allowed:
                os.environ[key] = value
        os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

    def _verify_prerequisites(self):
        # Validate before launching any node. Absolute Unicode values can be
        # corrupted when ROS launch serializes them into parameter YAML.
        for path in (self.scene.map_file, self.scene.entities_file,
                     self.points_file, self.path_matrix):
            if path is not None:
                ros_file_parameter(path)

        checks = {
            "ROS setup": ROS_SETUP.exists(),
            "workspace setup": WORKSPACE_SETUP.exists(),
            "workspace source": WORKSPACE_SRC.exists(),
            "static map": self.scene.map_file.exists(),
            f"{self.world} points": self.points_file.exists(),
            "scene manifest": self.scene.scene_file.exists(),
            "scene world": self.scene.world_file.exists(),
            "scene entities": self.scene.entities_file is None or self.scene.entities_file.exists(),
            "path matrix": self.path_matrix is None or self.path_matrix.exists(),
            "ros2 command": shutil.which("ros2") is not None,
        }
        if not self.args.no_foxglove:
            checks.update(
                {
                    "offline Foxglove launcher": FOXGLOVE_LAUNCHER.is_file(),
                    "offline Foxglove archive": FOXGLOVE_ARCHIVE.is_file()
                    and FOXGLOVE_ARCHIVE.stat().st_size > 0,
                    "rosbridge executable": Path(
                        "/opt/ros/humble/lib/rosbridge_server/rosbridge_websocket"
                    ).is_file(),
                    "rosapi executable": Path(
                        "/opt/ros/humble/lib/rosapi/rosapi_node"
                    ).is_file(),
                    "Ubuntu browser": any(
                        shutil.which(browser)
                        for browser in ("firefox", "chromium", "chromium-browser")
                    ),
                    "offline runtime commands": all(
                        shutil.which(command)
                        for command in ("curl", "sha256sum", "ss", "tar")
                    ),
                }
            )
        for name, ready in checks.items():
            self.log(f"[{'OK' if ready else 'MISSING'}] {name}")
        if not all(checks.values()):
            raise RuntimeError("Prerequisite check failed")
        validate_scene(self.scene, require_runtime_files=True)

    def _build_workspace(self):
        self.log("Building competition workspace...")
        # 2026-09-25：原来只建 6 个包，漏掉 tools_demo 与 IFRA_LinkAttacher 的两个包
        # （linkattacher_msgs / ros2_linkattacher），只跑过 --setup 的机器会缺
        # /ATTACHLINK / /DETACHLINK，抓取到吸附那一步静默失败。补齐成 9 个包。
        command = [
            "colcon",
            "build",
            "--symlink-install",
            "--build-base",
            str(workspace_build_base(WORKSPACE)),
            "--parallel-workers",
            "4",
            "--packages-select",
            "mybot_description",
            "mybot",
            "bot_navigation",
            "aic_perception",
            "aic_task",
            "aic_bringup",
            "linkattacher_msgs",
            "ros2_linkattacher",
            "tools_demo",
        ]
        subprocess.run(command, cwd=WORKSPACE, env=os.environ, check=True)
        self.log("Build completed.")

    def _create_run_dir(self):
        stamp = self.start_time.strftime("%Y%m%d_%H%M%S")
        base = log_base_dir()
        if not base.is_dir():
            raise RuntimeError(
                f"log base directory does not exist: {base}; "
                "create it first or point AIC_LOG_BASE at a writable directory"
            )
        self.run_dir = base / f"run_{stamp}"
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.log(f"run_dir={self.run_dir}")

    def _cleanup_residual(self):
        patterns = (
            "ros2 launch aic_bringup competition_world",
            "ros2 launch mybot gazebo_world",
            "ros2 launch bot_navigation nav_bringup_gazebo2.launch.py",
            "ros2 launch aic_perception grasp_pipeline.launch.py",
            "ros2 run aic_task task_parser",
            "ros2 run aic_task task_orchestrator",
            "ros2 run aic_task dynamic_obstacle",
            "ros2 run aic_task lifecycle_watchdog",
            "ros2 run aic_task tf_bridge",
            "ros2 launch mybot competition_move_group.launch.py",
            "ros2 run aic_task moveit_probe",
            "ros2 launch foxglove_bridge foxglove_bridge_launch.xml",
            "foxglove-studio",
            "start_offline_foxglove.sh",
            "ros2 launch rosbridge_server rosbridge_websocket_launch.xml",
            "/opt/ros/humble/lib/rosbridge_server/rosbridge_websocket",
            "/opt/ros/humble/lib/rosapi/rosapi_node",
            "http.server 8088",
            "http.server 8089",
            "http.server 8090",
            "yzbot-foxglove/profile-",
            "yzbot-foxglove",
            "/aic_perception/color_detector",
            "/aic_task/task_parser",
            "/aic_task/task_orchestrator",
            "/aic_task/dynamic_obstacle",
            "/aic_task/lifecycle_watchdog",
            "/aic_task/tf_bridge",
            "/move_group",
            "/competition_moveit_probe",
            "/foxglove_bridge",
            "/opt/ros/humble/lib/nav2_",
            "/opt/ros/humble/lib/tf2_ros/static_transform_publisher",
            "/opt/ros/humble/lib/robot_localization/ekf_node",
            "/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher",
            "/opt/ros/humble/lib/rviz2/rviz2",
            "gzserver",
            "gzclient",
        )
        for pattern in patterns:
            subprocess.run(
                ["pkill", "-INT", "-f", pattern],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if not any(self._process_pattern_exists(pattern) for pattern in patterns):
                break
            time.sleep(0.5)
        for pattern in patterns:
            subprocess.run(
                ["pkill", "-TERM", "-f", pattern],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if not any(self._process_pattern_exists(pattern) for pattern in patterns):
                return
            time.sleep(0.25)
        for pattern in patterns:
            subprocess.run(
                ["pkill", "-KILL", "-f", pattern],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )

    @staticmethod
    def _process_pattern_exists(pattern):
        result = subprocess.run(
            ["pgrep", "-f", pattern],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        return result.returncode == 0

    def _start_process(self, name, command):
        log_path = self.run_dir / f"{name}.log"
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=os.environ,
            cwd=WORKSPACE,
            start_new_session=True,
        )
        drain = _ChildLogDrain(name, log_path, CHILD_LOG_MAX_BYTES, process.stdout)
        drain.start()
        self.processes[name] = process
        self.log_handles[name] = drain
        self.log(f"[{name}] pid={process.pid} log={log_path}")
        return process

    def _launch_gazebo(self):
        command = ["ros2", "launch", "mybot", self.scene.launch_file]
        command.append(f"gui:={'False' if self.args.headless else 'True'}")
        if self.scene.entities_file is not None:
            command.append(f"entities_file:={ros_file_parameter(self.scene.entities_file)}")
        self._start_process(
            "gazebo",
            command,
        )

    def _launch_nav2(self):
        command = [
            "ros2",
            "launch",
            "bot_navigation",
            "nav_bringup_gazebo2.launch.py",
            f"map:={ros_file_parameter(self.scene.map_file)}",
            "slam:=False",
            f"rviz:={'False' if self.args.no_rviz else 'True'}",
        ]
        self._start_process("nav2", command)

    def _launch_moveit(self):
        self._start_process(
            "moveit",
            ["ros2", "launch", "mybot", "competition_move_group.launch.py"],
        )

    def _launch_dynamic_obstacles(self):
        self._start_process(
            "dynamic_obstacles",
            [
                "ros2",
                "run",
                "aic_task",
                "dynamic_obstacle",
                "--ros-args",
                "-p",
                "use_sim_time:=true",
                "-p",
                f"points_file:={ros_file_parameter(self.scene.obstacles_file or self.points_file)}",
                "-p",
                "enabled_obstacles:=" + ",".join(self.dynamic_obstacle_names),
            ],
        )

    def _launch_perception(self):
        self._start_process(
            "perception",
            [
                "ros2", "launch", "aic_perception", "grasp_pipeline.launch.py",
                "allow_motion:=true", "allow_attach:=true", "allow_detach:=true",
                f"candidates_config:={ros_file_parameter(self.points_file)}",
            ],
        )

    def _launch_task_system(self):
        if not self.args.no_task_parser:
            self._start_process(
                "task_parser",
                [
                    "ros2",
                    "run",
                    "aic_task",
                    "task_parser",
                    "--ros-args",
                    "-p",
                    f"mode:={self.mode}",
                ],
            )
        orchestrator_command = [
            "ros2",
            "run",
            "aic_task",
            "task_orchestrator",
            "--ros-args",
            "-p",
            f"mode:={self.mode}",
            "-p",
            f"parser_enabled:={str(not self.args.no_task_parser).lower()}",
            "-p",
            f"points_file:={ros_file_parameter(self.points_file)}",
            "-p",
            f"get_entity_service:={self.get_entity_service}",
            "-p",
            f"set_entity_service:={self.set_entity_service}",
            "-p",
            "development_cube_start_index:="
            f"{self.args.development_cube_start_index}",
        ]
        # The world2 slot-flow compares TF stamps with this node's ROS clock;
        # keep both in simulation time without changing v1/world3 launch behavior.
        if self.scene is not None and self.scene.scene_id == "world2":
            orchestrator_command.extend(["-p", "use_sim_time:=true"])
        if self.path_matrix is not None:
            orchestrator_command.extend(
                [
                    "-p",
                    f"path_matrix_file:={ros_file_parameter(self.path_matrix)}",
                    "-p",
                    "route_planning_required:=true",
                ]
            )
        if self.args.time_limit_sec is not None:
            orchestrator_command.extend(
                ["-p", f"time_limit_sec:={self.args.time_limit_sec}"]
            )
        if self.dynamic_obstacle_names and not self.args.no_dynamic_obstacles:
            orchestrator_command.extend(['-p', 'dynamic_obstacle_names:=' + ','.join(self.dynamic_obstacle_names)])
        self._start_process("task_orchestrator", orchestrator_command)

    def _launch_lifecycle_watchdog(self):
        """Start the runtime lifecycle watchdog (see aic_task/lifecycle_watchdog.py).

        Rationale: under this VM's load the Nav2 bond heartbeat can be missed for
        more than `bond_timeout` (4 s), after which the lifecycle manager
        deactivates the node and never brings it back. A deactivated
        `velocity_smoother` silently breaks the whole velocity chain, so the
        robot stops responding while every status topic still looks healthy.
        """
        if self.args.no_lifecycle_watchdog:
            self.stage_status["lifecycle_watchdog"] = "disabled_by_flag"
            self.log("[READY] lifecycle_watchdog: disabled_by_flag")
            return
        self._start_process(
            "lifecycle_watchdog",
            [
                "ros2",
                "run",
                "aic_task",
                "lifecycle_watchdog",
                "--ros-args",
                "-p",
                f"check_period_sec:={self.args.watchdog_period_sec}",
                "-p",
                # Nav2's own activation sequence can still be finishing when this
                # node starts (smoother_server / waypoint_follower come up late),
                # so give the stack time to settle before policing it.
                "startup_grace_sec:=20.0",
            ],
        )
        self._wait_process_alive("lifecycle_watchdog", 2.0)
        self._mark_stage("lifecycle_watchdog", "ready")

    def _launch_foxglove(self):
        runtime = WORKSPACE_SRC / "foxglove/start_offline_foxglove.sh"
        if not runtime.exists():
            raise RuntimeError(f"Offline Foxglove runtime missing: {runtime}")
        self._start_process("foxglove_runtime", ["bash", str(runtime)])
        self.stage_status["foxglove_studio"] = "offline_ubuntu"

    def _wait_gazebo_ready(self):
        self._wait_process_alive("gazebo", 5.0)
        self._wait_for_list_entries(
            "topic", ("/clock", "/scan", "/odom", "/joint_states"), 60.0
        )
        self._wait_for_list_entries(
            "service",
            ("/ATTACHLINK", "/DETACHLINK", self.get_entity_service),
            30.0,
        )
        self._wait_controllers(
            ("joint_state_broadcaster", "arm_controller", "gripper_controller"),
            45.0,
        )
        self._wait_topic_sample("/scan", 10.0)
        self._wait_topic_sample("/odom", 10.0)
        self._wait_topic_sample("/camera_rgbd/camera_rgbd/image_raw", 15.0)
        self._wait_topic_sample("/camera_rgbd/camera_rgbd/depth/image_raw", 15.0)
        return True

    def _wait_moveit_ready(self):
        self._wait_process_alive("moveit", 3.0)
        self._wait_for_list_entries("action", ("/move_action",), 45.0)
        self._wait_for_list_entries("service", ("/plan_kinematic_path",), 45.0)
        return True

    def _run_moveit_probe(self):
        log_path = self.run_dir / "moveit_probe.log"
        self.log(f"[moveit_probe] log={log_path}")
        with open(log_path, "w", buffering=1) as handle:
            result = subprocess.run(
                ["ros2", "run", "aic_task", "moveit_probe"],
                stdout=handle,
                stderr=subprocess.STDOUT,
                env=os.environ,
                text=True,
                timeout=75.0,
                check=False,
            )
        if result.returncode != 0:
            tail = "\n".join(log_path.read_text(errors="replace").splitlines()[-30:])
            raise RuntimeError(
                f"MoveIt planning/execution probe failed with code "
                f"{result.returncode}:\n{tail}"
            )
        self.moveit_probe_evidence = log_path.read_text(errors="replace").splitlines()
        return True

    def _wait_foxglove_ready(self):
        self._wait_process_alive("foxglove_runtime", 2.0)
        self._wait_for_list_entries(
            "node", ("/rosbridge_websocket", "/rosapi"), 25.0
        )
        # 只校验本次真正启用的窗口端口。原先写死 8088/8089/8090 三个，
        # 一旦 FOXGLOVE_WINDOWS 收窄到 dashboard，就绪门会永远等不到另外两个端口
        # 而报 FATAL（实测 19:01 那次启动）。
        windows = foxglove_selected_windows()
        http_ports = foxglove_window_ports(windows)
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            self._assert_process_alive("foxglove_runtime")
            try:
                for port in http_ports:
                    with socket.create_connection(("127.0.0.1", port), timeout=1.0):
                        pass
                with socket.create_connection(("127.0.0.1", rosbridge_port()), timeout=1.0):
                    break
            except OSError:
                time.sleep(0.5)
        else:
            raise RuntimeError(
                "Offline Foxglove did not listen on HTTP %s and ROSBridge %s (windows=%s)"
                % ("/".join(str(port) for port in http_ports), rosbridge_port(), ",".join(windows))
            )
        # 端口通了不等于"面板真的弹出来了"：浏览器窗口是**错开启动**的（默认每个
        # 间隔 20 s），而 snap Chromium 首次启动还有概率崩、脚本要换干净 profile 重试。
        # 所以必须等脚本自己给出结论行，否则会在窗口出现前几十秒就报 SYSTEM READY
        # （2026-09-24 实测：SYSTEM READY 17:52:41，看板窗口 17:53:2x 才上屏）。
        browser_deadline = time.monotonic() + 210.0
        while time.monotonic() < browser_deadline:
            self._assert_process_alive("foxglove_runtime")
            tail = self._read_log_tail("foxglove_runtime", 400)
            if "FOXGLOVE_BROWSER_FAILED" in tail or "browser_degraded=1" in tail:
                self.log(
                    "[WARN] Foxglove 浏览器窗口未能全部启动（browser_degraded=1）："
                    "面板可能不在屏幕上。可改用 SSH 端口转发在外部浏览器打开 "
                    "http://127.0.0.1:8090/?ds=rosbridge-websocket&ds.url="
                    f"ws%3A%2F%2F127.0.0.1%3A{rosbridge_port()}"
                )
                return "offline_ubuntu_ready_browser_degraded"
            if "OFFLINE_FOXGLOVE_READY" in tail:
                return "offline_ubuntu_ready"
            time.sleep(1.0)
        self.log(
            "[WARN] 等待浏览器窗口确认超时（210s）：面板可能还没上屏，请目视确认，"
            "或用 SSH 端口转发打开看板。"
        )
        return "offline_ubuntu_ready_browser_unconfirmed"

    def _wait_nav2_ready(self):
        self._wait_process_alive("nav2", 5.0)
        self._wait_for_list_entries("action", ("/navigate_to_pose",), 90.0)
        return self._wait_lifecycle_nodes(
            ("map_server", "amcl", "collision_monitor"), 60.0
        )

    def _enable_dynamic_obstacles(self):
        result = self._run_ros(['service', 'call', '/competition/start_dynamic_obstacles',
                                'std_srvs/srv/Trigger', '{}'], timeout=10.0)
        if result.returncode != 0 or 'success=True' not in result.stdout:
            raise RuntimeError('dynamic obstacle motion enable was not confirmed')

    def _wait_dynamic_obstacles_configured(self):
        self._wait_process_alive('dynamic_obstacles', 2.0)
        self._wait_for_list_entries('node', ('/dynamic_obstacle_controller',), 15.0)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            self._assert_process_alive('dynamic_obstacles')
            if 'configured; awaiting READY' in self._read_log_tail('dynamic_obstacles'):
                for name in self.dynamic_obstacle_names:
                    self._get_entity_xy(name)
                return True
            time.sleep(.2)
        raise RuntimeError('dynamic obstacle configuration was not confirmed')

    def _wait_dynamic_obstacles_ready(self):
        self._wait_process_alive("dynamic_obstacles", 2.0)
        self._wait_for_list_entries("node", ("/dynamic_obstacle_controller",), 15.0)
        names = self.dynamic_obstacle_names
        initial = {name: self._get_entity_xy(name) for name in names}
        deadline = time.monotonic() + 12.0
        latest = initial
        max_movement = {name: 0.0 for name in names}
        max_position = dict(initial)
        while time.monotonic() < deadline:
            self._assert_process_alive("dynamic_obstacles")
            time.sleep(1.0)
            latest = {name: self._get_entity_xy(name) for name in names}
            movement = {
                name: math.hypot(
                    latest[name][0] - initial[name][0],
                    latest[name][1] - initial[name][1],
                )
                for name in names
            }
            for name, distance in movement.items():
                if distance > max_movement[name]:
                    max_movement[name] = distance
                    max_position[name] = latest[name]
            if all(distance >= 0.05 for distance in max_movement.values()):
                self.dynamic_obstacle_samples = {
                    name: {
                        "initial_xy": list(initial[name]),
                        "latest_xy": list(max_position[name]),
                        "movement_m": max_movement[name],
                    }
                    for name in names
                }
                self.log(
                    f"[dynamic_obstacles] verified max_movement={max_movement}"
                )
                return True
        raise RuntimeError(
            "Dynamic obstacles did not move enough: "
            f"initial={initial}, latest={latest}, max_movement={max_movement}"
        )

    def _get_entity_xy(self, name):
        request = f"{{name: {name}, reference_frame: world}}"
        last_error = "no response"
        for attempt in range(1, 4):
            result = self._run_ros(
                [
                    "service",
                    "call",
                    self.get_entity_service,
                    "gazebo_msgs/srv/GetEntityState",
                    request,
                ],
                timeout=8.0,
            )
            if result.returncode == 0 and "success=True" in result.stdout:
                match = re.search(
                    r"position=.*?x=([-+0-9.eE]+),\s*y=([-+0-9.eE]+)",
                    result.stdout,
                    re.DOTALL,
                )
                if match is not None:
                    return float(match.group(1)), float(match.group(2))
                last_error = "response position could not be parsed"
            else:
                last_error = (result.stderr or result.stdout).strip()
            self.log(
                f"[gazebo_entity] retry={attempt}/3 name={name} error={last_error}"
            )
            time.sleep(0.5)
        raise RuntimeError(f"Unable to read Gazebo entity {name}: {last_error}")

    def _reset_robot_initial_pose(self):
        x, y, yaw = self.initial_pose
        half_yaw = yaw / 2.0
        request = (
            "{state: {name: six_arm, pose: {position: "
            f"{{x: {x}, y: {y}, z: 0.0}}, orientation: "
            f"{{x: 0.0, y: 0.0, z: {math.sin(half_yaw)}, "
            f"w: {math.cos(half_yaw)}}}}}, twist: {{linear: "
            "{x: 0.0, y: 0.0, z: 0.0}, angular: "
            "{x: 0.0, y: 0.0, z: 0.0}}, reference_frame: world}}"
        )
        result = self._run_ros(
            [
                "service",
                "call",
                self.set_entity_service,
                "gazebo_msgs/srv/SetEntityState",
                request,
            ],
            timeout=10.0,
        )
        if result.returncode != 0 or "success=True" not in result.stdout:
            raise RuntimeError(
                "Unable to reset robot initial pose: "
                + (result.stderr or result.stdout).strip()
            )
        time.sleep(2.0)
        actual_x, actual_y = self._get_entity_xy("six_arm")
        error = math.hypot(actual_x - x, actual_y - y)
        self.robot_initial_pose_sample = {
            "requested_xy": [x, y],
            "actual_xy": [actual_x, actual_y],
            "error_m": error,
        }
        self.log(
            f"[robot_initial_pose] actual=({actual_x:.4f}, {actual_y:.4f}) "
            f"error={error:.4f}m"
        )
        if error > 0.03:
            raise RuntimeError(
                f"Robot initial pose drifted {error:.3f} m after reset"
            )
        return True

    def _wait_navigation_active(self):
        return self._wait_lifecycle_nodes(NAV2_LIFECYCLE_NODES, 60.0)

    def _wait_lifecycle_nodes(self, nodes, timeout):
        deadline = time.monotonic() + timeout
        pending = set(nodes)
        while pending and time.monotonic() < deadline:
            self._assert_process_alive("nav2")
            for node in tuple(pending):
                result = self._run_ros(
                    ["lifecycle", "get", f"/{node}"], timeout=5.0
                )
                if result.returncode == 0 and "active [3]" in result.stdout:
                    pending.remove(node)
                elif result.returncode == 0 and "unconfigured [1]" in result.stdout:
                    self._run_ros(
                        ["lifecycle", "set", f"/{node}", "configure"],
                        timeout=12.0,
                    )
                elif result.returncode == 0 and "inactive [2]" in result.stdout:
                    self._run_ros(
                        ["lifecycle", "set", f"/{node}", "activate"],
                        timeout=20.0,
                    )
            if pending:
                self.log(f"[nav2] waiting lifecycle={sorted(pending)}")
                time.sleep(2.0)
        if pending:
            raise RuntimeError(f"Nav2 lifecycle timeout: {sorted(pending)}")
        return True

    def _set_initial_pose(self):
        x, y, yaw = self.initial_pose
        half_yaw = yaw / 2.0
        message = (
            "{header: {frame_id: map}, pose: {pose: {position: "
            f"{{x: {x}, y: {y}, z: 0.0}}, orientation: "
            f"{{x: 0.0, y: 0.0, z: {math.sin(half_yaw)}, "
            f"w: {math.cos(half_yaw)}}}}}, covariance: "
            "[0.01, 0.0, 0.0, 0.0, 0.0, 0.0, "
            "0.0, 0.01, 0.0, 0.0, 0.0, 0.0, "
            "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
            "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
            "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
            "0.0, 0.0, 0.0, 0.0, 0.0, 0.01]}}"
        )
        result = self._run_ros(
            [
                "topic",
                "pub",
                "--rate",
                "2",
                "--times",
                "10",
                "/initialpose",
                "geometry_msgs/msg/PoseWithCovarianceStamped",
                message,
            ],
            timeout=12.0,
        )
        if result.returncode != 0:
            raise RuntimeError(f"Initial pose publish failed: {result.stderr.strip()}")

    def _wait_localization_ready(self):
        self._wait_topic_sample("/odom", 15.0)
        for attempt in range(1, 4):
            try:
                self._wait_topic_sample("/amcl_pose", 10.0)
            except RuntimeError:
                self.log(f"[localization] AMCL pose retry {attempt}/3")
                self._set_initial_pose()
                continue

            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                result = self._run_ros(
                    ["run", "tf2_ros", "tf2_echo", "map", "base_footprint"],
                    timeout=4.0,
                )
                if "Translation:" in result.stdout:
                    return True
                time.sleep(1.0)
            self.log(f"[localization] map TF retry {attempt}/3")
            self._set_initial_pose()
        raise RuntimeError("Full map->odom->base_footprint TF was not available")

    def _check_tf_integrity(self):
        result = self._run_ros(["topic", "info", "/tf", "--verbose"], timeout=10.0)
        if result.returncode != 0:
            raise RuntimeError("Unable to inspect /tf publishers")
        forbidden = ("tf_bridge", "map_tf_publisher")
        present = [name for name in forbidden if f"Node name: {name}" in result.stdout]
        if present:
            raise RuntimeError(f"Duplicate TF publishers detected: {present}")
        if "Node name: differential_drive_controller" not in result.stdout:
            raise RuntimeError("Authoritative odom TF publisher is missing")
        if "Node name: amcl" not in result.stdout:
            raise RuntimeError("Authoritative AMCL map->odom TF publisher is missing")
        amcl_parameter = None
        for attempt in range(1, 4):
            amcl_parameter = self._run_ros(
                ["param", "get", "/amcl", "tf_broadcast"], timeout=8.0
            )
            if (
                amcl_parameter.returncode == 0
                and "true" in amcl_parameter.stdout.lower()
            ):
                break
            self.log(
                f"[tf_integrity] AMCL parameter retry={attempt}/3 "
                f"returncode={amcl_parameter.returncode}"
            )
            time.sleep(1.0)
        else:
            detail = (amcl_parameter.stderr or amcl_parameter.stdout).strip()
            raise RuntimeError(f"AMCL tf_broadcast is not enabled: {detail}")
        static_tf = self._run_ros(
            ["topic", "info", "/tf_static", "--verbose"], timeout=10.0
        )
        if "Node name: map_to_odom_static" in static_tf.stdout:
            raise RuntimeError("Competing static map->odom publisher is present")
        return True

    def _perception_diagnosis(self):
        """感知就绪门超时时收集证据：谁死了、相机话题还有没有发布者。

        2026-09-24 实跑：启动期 `gazebo` 偶发 exit 255（相机插件随之消失），
        表现为 `FATAL: No message received from /competition/detection_image`，
        但原始日志里看不到"是谁挂了"，只能靠猜。这里把关键证据一次打全。
        """
        parts = []
        for name in ("gazebo", "perception"):
            process = self.processes.get(name)
            if process is None:
                parts.append(f"{name}=未管理")
            elif process.poll() is None:
                parts.append(f"{name}=alive(pid={process.pid})")
            else:
                parts.append(f"{name}=EXITED(code={process.returncode})")
        for topic in (
            "/camera_rgbd/camera_rgbd/image_raw",
            "/competition/detection_image",
        ):
            result = self._run_ros(["topic", "info", topic], timeout=6.0)
            match = re.search(r"Publisher count:\s*(\d+)", result.stdout or "")
            parts.append(
                f"{topic} 发布者={int(match.group(1)) if match else '?'}"
            )
        for name in ("gazebo", "perception"):
            if name in self.processes:
                tail = self._read_log_tail(name, 3).strip().replace("\n", " | ")
                parts.append(f"{name} 日志尾={tail[-240:]!r}")
        return "；".join(parts)

    def _wait_perception_ready(self):
        self._wait_process_alive("perception", 3.0)
        self._wait_for_list_entries(
            "topic",
            (
                "/competition/detection_image",
                "/competition/detections_json",
                "/competition/grasp_point",
            ),
            20.0,
        )
        # 首轮不等到图就重试一次：实测首轮失败有两种成因 —— ① gazebo 偶发崩溃
        # （相机插件没了，重试也没用，但要打清楚证据）；② DDS 首轮发现偏慢/相机
        # 初始化慢（重试就能过）。这里重试一次，两次都不行才 FATAL。
        try:
            self._wait_topic_sample("/competition/detection_image", 20.0)
        except RuntimeError as exc:
            diagnosis = self._perception_diagnosis()
            self.log(f"[WARN] {exc}；诊断：{diagnosis}")
            self.log("[WARN] 感知首轮超时 → 重试一次（30 s）")
            try:
                self._wait_topic_sample("/competition/detection_image", 30.0)
            except RuntimeError as retry_exc:
                raise RuntimeError(
                    f"{retry_exc}（已重试一次）。诊断：{self._perception_diagnosis()}"
                ) from retry_exc
        self._wait_topic_sample("/competition/detections_json", 20.0)
        self._wait_for_list_entries(
            "service",
            (
                "/competition/set_target_model",
                "/competition/reset_pick",
                "/competition/execute_pick_sequence",
                "/competition/release_target",
                "/competition/abort_pick",
                "/competition/stop_pick_sequence",
                "/competition/reset_pick_sequence",
                "/competition/stop_grasp",
            ),
            30.0,
        )
        return True

    def _wait_task_ready(self):
        if not self.args.no_task_parser:
            self._wait_process_alive("task_parser", 2.0)
        self._wait_process_alive("task_orchestrator", 2.0)
        expected_topics = [
            "/competition/start",
            "/competition/current_state",
            "/competition/current_group_count",
            "/competition/task_plan_json",
        ]
        if not self.args.no_task_parser:
            expected_topics.append("/competition/resubmit")
            self._wait_for_list_entries("service", (
                "/competition/cancel_task", "/competition/clear_task", "/competition/freeze_task"), 20.0)
        self._wait_for_list_entries(
            "topic",
            tuple(expected_topics),
            20.0,
        )
        deadline = time.monotonic() + 45.0
        while time.monotonic() < deadline:
            self._assert_process_alive("task_orchestrator")
            # 先读日志：`topic echo --once` 要求「下一条样本」，而编排器的
            # 重复状态心跳已按 test13 性能记录 §6 降到 30 s（状态变化仍立即
            # 发布）。READY 之后如果没有新的状态迁移，`--once` 会一直等不到，
            # 于是启动器把健康的栈判成失败。日志里的 ready 行是权威且无竞态的
            # 证据（test13 的启动器同样读日志而不是 echo 话题）。
            tail = self._read_log_tail("task_orchestrator", 400)
            if "TASK_FAILED" in tail or "INITIALIZATION_FAILED" in tail:
                raise RuntimeError(
                    "Task orchestrator entered TASK_FAILED during startup. Log tail:\n"
                    + tail
                )
            if "Task orchestrator ready:" in tail or "State: BOOTING -> READY" in tail:
                return True
            # 话题作为次要信号保留：若恰好等到了样本也接受。
            result = self._run_ros(
                ["topic", "echo", "/competition/current_state", "--once"],
                timeout=3.0,
            )
            state_match = re.search(r"data:\s*['\"]?([A-Z_]+)", result.stdout)
            state = state_match.group(1) if state_match is not None else None
            if state == "READY":
                return True
            if state in ("TASK_FAILED", "INITIALIZATION_FAILED"):
                raise RuntimeError(
                    "Task orchestrator entered TASK_FAILED during startup. Log tail:\n"
                    + self._read_log_tail("task_orchestrator")
                )
            time.sleep(0.5)
        raise RuntimeError(
            "Task orchestrator did not reach READY during startup. Log tail:\n"
            + self._read_log_tail("task_orchestrator")
        )

    def _wait_process_alive(self, name, delay):
        time.sleep(delay)
        self._assert_process_alive(name)

    def _assert_process_alive(self, name):
        process = self.processes[name]
        if process.poll() is not None:
            tail = self._read_log_tail(name)
            raise RuntimeError(
                f"{name} exited with code {process.returncode}. Log tail:\n{tail}"
            )

    def _wait_for_list_entries(self, kind, expected, timeout):
        deadline = time.monotonic() + timeout
        command = [kind, "list"]
        missing = set(expected)
        while time.monotonic() < deadline:
            result = self._run_ros(command, timeout=8.0)
            if result.returncode == 0:
                entries = set(result.stdout.splitlines())
                missing = set(expected) - entries
                if not missing:
                    return
            for name in tuple(self.processes):
                if name != "foxglove_studio":
                    self._assert_process_alive(name)
            self.log(f"[{kind}] waiting={sorted(missing)}")
            time.sleep(2.0)
        raise RuntimeError(f"Timed out waiting for {kind} entries: {sorted(missing)}")

    def _wait_controllers(self, expected, timeout):
        deadline = time.monotonic() + timeout
        missing = set(expected)
        while time.monotonic() < deadline:
            result = self._run_ros(["control", "list_controllers"], timeout=8.0)
            active = set()
            if result.returncode == 0:
                for line in result.stdout.splitlines():
                    for controller in expected:
                        if controller in line and "active" in line:
                            active.add(controller)
            missing = set(expected) - active
            if not missing:
                return
            self._assert_process_alive("gazebo")
            self.log(f"[controllers] waiting={sorted(missing)}")
            time.sleep(2.0)
        raise RuntimeError(f"Controller activation timeout: {sorted(missing)}")

    def _wait_topic_sample(self, topic, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            result = self._run_ros(["topic", "echo", topic, "--once"], timeout=4.0)
            if result.returncode == 0 and result.stdout.strip():
                return
            time.sleep(0.5)
        raise RuntimeError(f"No message received from {topic}")

    def _run_ros(self, arguments, timeout):
        try:
            return subprocess.run(
                ["ros2", *arguments],
                capture_output=True,
                text=True,
                timeout=timeout,
                env=os.environ,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            stdout = exc.stdout or ""
            stderr = exc.stderr or "timeout"
            if isinstance(stdout, bytes):
                stdout = stdout.decode(errors="replace")
            if isinstance(stderr, bytes):
                stderr = stderr.decode(errors="replace")
            return subprocess.CompletedProcess(
                exc.cmd, 124, stdout, stderr
            )

    def _mark_stage(self, name, ready):
        if not ready:
            raise RuntimeError(f"Stage failed: {name}")
        status = ready if isinstance(ready, str) else "ready"
        self.stage_status[name] = status
        self.log(f"[READY] {name}: {status}")
        self._write_manifest()

    def _start_heartbeat(self):
        heartbeat_path = self.run_dir / "heartbeat.log"

        def heartbeat_loop():
            counter = 0
            while not self._heartbeat_stop.wait(5.0):
                line = datetime.now().astimezone().isoformat()
                with open(heartbeat_path, "a") as handle:
                    handle.write(line + "\n")
                counter += 1
                if counter % 6 == 0:
                    self.log(f"heartbeat stages={self.stage_status}")

        self._heartbeat_thread = threading.Thread(target=heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()

    def _write_manifest(self):
        if self.run_dir is None:
            return
        manifest = {
            "start_time": self.start_time.astimezone().isoformat(),
            "updated_at": datetime.now().astimezone().isoformat(),
            "mode": self.mode,
            "world": self.world,
            "world2_docking_profile": (
                fixed_world2_parameters() if self.world == "world2" else None
            ),
            "ros_domain_id": self.ros_domain_id,
            "time_limit_sec_override": self.args.time_limit_sec,
            "ros_localhost_only": os.environ.get("ROS_LOCALHOST_ONLY", "0") == "1",
            "map_yaml": str(self.scene.map_file),
            "scene_selection_source": self.scene.selection_source,
            "scene_status": self.scene.status,
            "scene_file": str(self.scene.scene_file),
            "scene_entities_file": str(self.scene.entities_file) if self.scene.entities_file else None,
            "scene_world_file": str(self.scene.world_file),
            "scene_asset_sha256": self.scene.asset_hashes(),
            "points_file": str(self.points_file),
            "path_matrix_file": str(self.path_matrix) if self.path_matrix else None,
            "dynamic_obstacle_names": list(self.dynamic_obstacle_names),
            "lifecycle_watchdog": (
                "disabled" if self.args.no_lifecycle_watchdog else "enabled"
            ),
            "lifecycle_watchdog_period_sec": (
                None if self.args.no_lifecycle_watchdog else self.args.watchdog_period_sec
            ),
            "initial_pose": list(self.initial_pose),
            "stages": self.stage_status,
            "dynamic_obstacle_samples": getattr(
                self, "dynamic_obstacle_samples", None
            ),
            "robot_initial_pose_sample": getattr(
                self, "robot_initial_pose_sample", None
            ),
            "moveit_probe_evidence": getattr(self, "moveit_probe_evidence", None),
            "foxglove_websocket": (
                None if self.args.no_foxglove else f"ws://127.0.0.1:{rosbridge_port()}"
            ),
            "foxglove_windows": (
                None
                if self.args.no_foxglove
                else {
                    window: "http://127.0.0.1:%d/?ds=rosbridge-websocket" % port
                    for window, port in zip(
                        foxglove_selected_windows(),
                        foxglove_window_ports(),
                    )
                }
            ),
            "degraded_subsystems": dict(self._degraded_subsystems),
            "processes": {
                name: {"pid": process.pid, "exit_code": process.poll()}
                for name, process in self.processes.items()
            },
        }
        temporary = self.run_dir / "run_manifest.json.tmp"
        final = self.run_dir / "run_manifest.json"
        temporary.write_text(json.dumps(manifest, indent=2) + "\n")
        temporary.replace(final)

    def _read_log_tail(self, name, line_count=25):
        if self.run_dir is None:
            return ""
        path = self.run_dir / f"{name}.log"
        if not path.exists():
            return ""
        return "\n".join(path.read_text(errors="replace").splitlines()[-line_count:])

    def _wait_shutdown(self):
        def request_shutdown(received_signal, _frame):
            # Signal handlers must not perform process cleanup or acquire locks.
            self._shutdown_signal = received_signal
            self._shutdown_requested = True

        signal.signal(signal.SIGINT, request_shutdown)
        signal.signal(signal.SIGTERM, request_shutdown)
        while not self._shutdown_requested:
            for name, process in self.processes.items():
                if name in FOXGLOVE_OPTIONAL_PROCESSES:
                    # 可视化辅助进程不是安全相关子系统，它挂了不能判比赛失败。
                    # 实测：Foxglove 的 dashboard 会遍历 /rosapi/node_details，
                    # Humble 的 rosapi 在枚举途中遇到节点消失会抛
                    # `TypeError: cannot unpack non-iterable NoneType object`
                    # 而退出，连锁把 rosbridge 和本启动脚本一起带走 —— 原本的
                    # 结果是整场 run 被 FATAL 掉，尽管比赛链完全健康。
                    # 这里只告警一次并继续，把状态记进 run manifest。
                    if process.poll() is not None:
                        if name not in self._degraded_subsystems:
                            self._degraded_subsystems[name] = process.returncode
                            self.log(
                                f"[WARN] visualisation subsystem {name} exited "
                                f"with code {process.returncode}; continuing the "
                                "match without it"
                            )
                    continue
                if process.poll() is not None:
                    raise RuntimeError(
                        f"Required subsystem {name} exited with code {process.returncode}"
                    )
            time.sleep(0.2)
        self.log(f"Shutdown requested by signal {self._shutdown_signal}.")

    def _shutdown(self):
        if self._stopping:
            return
        self._stopping = True
        self._heartbeat_stop.set()
        self.log("Stopping managed subsystems...")
        for process in self.processes.values():
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            if all(process.poll() is not None for process in self.processes.values()):
                break
            time.sleep(0.25)
        for process in self.processes.values():
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        self._cleanup_residual()
        if self._disk_guard is not None:
            self._disk_guard.close()
        for name, handle in self.log_handles.items():
            if getattr(handle, "dropped", 0):
                self.log(f"[{name}] log {handle.summary()}")
            handle.close()
        self.stage_status["shutdown"] = "complete"
        self._write_manifest()
        self.log("All managed subsystems stopped.")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode", choices=("competition", "development"), default="competition"
    )
    parser.add_argument(
        "--world", default="auto",
        help="scene ID under aic_task/config/scenes; auto reads scenes/active.yaml",
    )
    parser.add_argument("--display", default=":0")
    parser.add_argument("--ros-domain-id", type=int, default=42)
    parser.add_argument("--initial-pose-x", type=float, default=INITIAL_POSE[0])
    parser.add_argument("--initial-pose-y", type=float, default=INITIAL_POSE[1])
    parser.add_argument("--initial-pose-yaw", type=float, default=INITIAL_POSE[2])
    parser.add_argument(
        "--time-limit-sec",
        type=float,
        help="diagnostic override; the competition default remains 300 seconds",
    )
    parser.add_argument("--no-foxglove", action="store_true")
    parser.add_argument("--no-rviz", action="store_true")
    parser.add_argument(
        "--no-dynamic-obstacles",
        action="store_true",
        help="disable World3 moving obstacles during static cube acceptance",
    )
    parser.add_argument(
        "--dynamic-obstacle-names",
        default="all",
        help="comma-separated moving obstacles for the selected world; default: all",
    )
    parser.add_argument(
        "--no-task-parser",
        action="store_true",
        help="disable question generation and accept a directly published audit plan",
    )
    parser.add_argument(
        "--no-lifecycle-watchdog",
        action="store_true",
        help=(
            "disable the runtime lifecycle watchdog (ablation switch; default: enabled)"
        ),
    )
    parser.add_argument(
        "--watchdog-period-sec",
        type=float,
        default=1.0,
        help="lifecycle watchdog poll period in seconds (default: 1.0)",
    )
    parser.add_argument(
        "--development-cube-start-index",
        type=int,
        default=0,
        help="zero-based cube index used by deterministic development audits",
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--reset", action="store_true")
    parser.add_argument("--setup", action="store_true")
    args = parser.parse_args()
    if args.time_limit_sec is not None and args.time_limit_sec <= 0.0:
        parser.error("--time-limit-sec must be positive")
    if not args.reset and args.dynamic_obstacle_names != "all":
        selected = {
            name.strip()
            for name in args.dynamic_obstacle_names.split(",")
            if name.strip()
        }
        try:
            available = load_scene(args.world, SCENES_ROOT, WORKSPACE).dynamic_obstacles
        except ValueError as exc:
            parser.error(str(exc))
        unknown = sorted(selected - set(available))
        if not selected or unknown:
            parser.error(
                "--dynamic-obstacle-names must be all or a comma-separated "
                f"subset of {list(available)}; unknown={unknown}"
            )
    if not 0 <= args.development_cube_start_index <= 4:
        parser.error("--development-cube-start-index must be in [0, 4]")
    if args.development_cube_start_index and args.mode != "development":
        parser.error("--development-cube-start-index requires --mode development")
    if (
        (args.initial_pose_x, args.initial_pose_y, args.initial_pose_yaw)
        != INITIAL_POSE
        and args.mode != "development"
    ):
        parser.error("custom initial pose requires --mode development")
    return args


def main():
    launcher = None
    try:
        launcher = CompetitionLauncher(parse_args())
        launcher.run()
    except KeyboardInterrupt:
        if launcher is not None:
            launcher._shutdown()
    except Exception as exc:
        print(f"FATAL: {exc}", file=sys.stderr, flush=True)
        if launcher is not None:
            launcher._shutdown()
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
