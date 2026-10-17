"""Plan O: 避障量化探针（只读，不干预栈）。

用途：在改任何避障参数之前，先能量出"现在到底有多危险"，改完再用同一把尺子对比。

采样与指标：
  - 10 Hz 记录（内部按每条 /gazebo/model_states 计算，最细粒度跟踪最小值）
  - **表面间隙** = 机器人基座中心 到 移动方块**有向矩形**的距离 − 机器人半径
      * 负值 ⇒ 两者足迹重叠 ⇒ 真的撞上/压上
      * 0.4 m 以上一般算安全，< 0.15 m 属危险接近
  - 每个作业项（item）统计：最小间隙、<0.30 m 次数、<0.15 m 次数、重叠次数、
    最近一次的最小间隙时车速、期间碰撞监视器触发次数
  - 全局统计 + 碰撞监视器状态时长
  - 结果写 JSON（含 10 Hz 采样序列），终端每 30 s 打一行进度

运行（另开一个终端，栈已启动时）：
  source /opt/ros/humble/setup.bash
  source /home/ros/桌面/AIC_Competition_Work/runtime/312e163/dev_ws/install/setup.bash
  export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  python3 scratch/plan_o_avoidance_probe.py \
      --out /home/ros/桌面/AIC_Competition_Work/run_records/<记录>/results/avoidance_before.json
"""
from pathlib import Path
import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String

try:
    from nav2_msgs.msg import CollisionMonitorState
except Exception:  # pragma: no cover - message may be absent in some installs
    CollisionMonitorState = None

try:
    import yaml
except Exception:  # pragma: no cover
    yaml = None

DEFAULT_SCENE = ("/home/ros/桌面/AIC_Competition_Work/runtime/312e163/dev_ws/src/yzbot/"
                 "aic_task/config/scenes/world2/obstacles.yaml")

MONITOR_NAMES = {}
if CollisionMonitorState is not None:
    for _name in ("DO_NOTHING", "STOP", "SLOWDOWN", "APPROACH", "LIMIT"):
        _value = getattr(CollisionMonitorState, _name, None)
        if isinstance(_value, int):
            MONITOR_NAMES[_value] = _name
    # 兜底：Humble 里常量顺序为 DO_NOTHING=0, STOP=1, SLOWDOWN=2, APPROACH=3, LIMIT=4
    for _value, _name in ((0, "DO_NOTHING"), (1, "STOP"), (2, "SLOWDOWN"),
                          (3, "APPROACH"), (4, "LIMIT")):
        MONITOR_NAMES.setdefault(_value, _name)


def yaw_of(pose):
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rect_clearance(px, py, box_pose, half_w, half_h, robot_radius):
    """机器人圆盘（中心 px,py，半径 robot_radius）到有向矩形的表面间隙（可为负）。"""
    dx = px - box_pose.position.x
    dy = py - box_pose.position.y
    yaw = yaw_of(box_pose)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    # 把点变到方块坐标系（旋转 −yaw）
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    outside_x = max(abs(local_x) - half_w, 0.0)
    outside_y = max(abs(local_y) - half_h, 0.0)
    return math.hypot(outside_x, outside_y) - robot_radius


class AvoidanceProbe(Node):
    def __init__(self, args):
        super().__init__("plan_o_avoidance_probe")
        self.args = args
        self.boxes = self._load_boxes(args.scene)
        self.t0 = time.monotonic()
        self.last_record = 0.0
        self.last_progress = 0.0
        self.samples = []
        self.items = []
        self.state = ""
        self.in_item = False
        self.item_start = None
        self.item_completed = 0
        self.cur = None
        self.monitor_state = None
        self.monitor_last_change = None
        self.monitor_durations = {}
        self.monitor_events = []
        self.monitor_activations_total = 0
        self.overall = self._blank_bucket()

        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(ModelStates, "/gazebo/model_states", self._on_models, qos)
        self.create_subscription(String, "/competition/current_state", self._on_state, 10)
        if CollisionMonitorState is not None:
            self.create_subscription(CollisionMonitorState, "/collision_monitor_state",
                                     self._on_monitor, 10)
        self.get_logger().info(
            f"探针启动：机器人={args.robot} 方块={list(self.boxes)} 阈值 "
            f"near={args.near} danger={args.danger} 输出={args.out}")

    # ---------- 配置与统计容器 ----------
    def _load_boxes(self, scene_path):
        boxes = {name: (0.19, 0.20) for name in self.args.boxes.split(",")}
        if yaml is None:
            return boxes
        try:
            data = yaml.safe_load(Path(scene_path).read_text()) or {}
            for obstacle in data.get("dynamic_obstacles", []) or []:
                name = str(obstacle.get("name"))
                dims = obstacle.get("dimensions") or [0.38, 0.4, 0.3]
                if name in boxes and len(dims) >= 2:
                    boxes[name] = (float(dims[0]) / 2.0, float(dims[1]) / 2.0)
        except Exception as exc:  # noqa: BLE001 - 探针不能因配置问题挂掉
            self.get_logger().warning(f"读取 {scene_path} 失败，用默认尺寸：{exc}")
        return boxes

    def _blank_bucket(self):
        return {name: {"min_clearance": None, "near": 0, "danger": 0, "contact": 0,
                       "speed_at_min": None, "min_time": None}
                for name in self.boxes}

    def _reset_item(self):
        self.item_start = time.monotonic()
        self.cur = self._blank_bucket()
        self.cur_monitor_activations = 0

    # 这些状态视为"作业项之间"；其余状态（前往物块/抓取/搬运/放置…）算在一个 item 内
    BOUNDARY_STATES = ("", "BOOTING", "READY", "ITEM_SUCCEEDED", "ITEM_FAILED",
                       "TASK_SUCCEEDED", "TASK_FAILED")

    # ---------- 订阅回调 ----------
    def _on_state(self, msg):
        previous, self.state = self.state, msg.data
        working = self.state not in self.BOUNDARY_STATES
        if working and not self.in_item:
            self.in_item = True
            self._reset_item()
            self.get_logger().info(f"item 开始（state={self.state}）")
        elif self.in_item and not working:
            self._finish_item(reason=f"state->{self.state}")

    def _on_monitor(self, msg):
        value = int(getattr(msg, "action_type", -1))
        name = MONITOR_NAMES.get(value, str(value))
        now = time.monotonic()
        if self.monitor_last_change is not None and self.monitor_state is not None:
            self.monitor_durations[self.monitor_state] = (
                self.monitor_durations.get(self.monitor_state, 0.0)
                + (now - self.monitor_last_change))
        if name != self.monitor_state:
            self.monitor_events.append({"t": round(now - self.t0, 2), "state": name,
                                        "polygon": str(getattr(msg, "polygon_name", ""))})
            if name not in ("DO_NOTHING",):
                self.monitor_activations_total += 1
                if self.in_item:
                    self.cur_monitor_activations += 1
        self.monitor_state = name
        self.monitor_last_change = now

    def _on_models(self, msg):
        index = {name: i for i, name in enumerate(msg.name)}
        if self.args.robot not in index:
            return
        now = time.monotonic()
        robot_pose = msg.pose[index[self.args.robot]]
        twist = msg.twist[index[self.args.robot]]
        speed = math.hypot(twist.linear.x, twist.linear.y)
        values = {}
        for name, (half_w, half_h) in self.boxes.items():
            if name not in index:
                continue
            clearance = rect_clearance(robot_pose.position.x, robot_pose.position.y,
                                       msg.pose[index[name]], half_w, half_h,
                                       self.args.robot_radius)
            values[name] = clearance
            self._track(name, clearance, speed, now)
        if now - self.last_record >= 0.1:
            self.last_record = now
            self.samples.append([round(now - self.t0, 2),
                                 {k: round(v, 3) for k, v in values.items()},
                                 round(speed, 3), self.state])
            if len(self.samples) > 200:
                del self.samples[:-200]
        if now - self.last_progress >= 30.0:
            self.last_progress = now
            self._print_progress()

    def _track(self, name, clearance, speed, now):
        for bucket in (self.overall, self.cur):
            if bucket is None:
                continue
            entry = bucket[name]
            if entry["min_clearance"] is None or clearance < entry["min_clearance"]:
                entry["min_clearance"] = clearance
                entry["speed_at_min"] = speed
                entry["min_time"] = round(now - self.t0, 2)
            if clearance < self.args.near:
                entry["near"] += 1
            if clearance < self.args.danger:
                entry["danger"] += 1
            if clearance < 0.0:
                entry["contact"] += 1

    # ---------- 输出 ----------
    def _print_progress(self):
        parts = []
        for name, entry in self.overall.items():
            value = entry["min_clearance"]
            parts.append(f"{name}: min={'-' if value is None else f'{value:.3f}'} "
                         f"<{self.args.near}={entry['near']} <{self.args.danger}={entry['danger']} "
                         f"contact={entry['contact']}")
        line = " | ".join(parts)
        self.get_logger().info(
            f"[{time.monotonic() - self.t0:6.1f}s] {line} | 监视器触发={self.monitor_activations_total} "
            f"当前={self.monitor_state}")

    def _finish_item(self, reason):
        if not self.in_item:
            return
        record = {
            "item": len(self.items) + 1,
            "started_s": round((self.item_start or time.monotonic()) - self.t0, 2),
            "ended_s": round(time.monotonic() - self.t0, 2),
            "end_reason": reason,
            "monitor_activations": getattr(self, "cur_monitor_activations", 0),
            "boxes": self.cur,
        }
        self.items.append(record)
        parts = []
        for name, entry in self.cur.items():
            value = entry["min_clearance"]
            shown = "-" if value is None else f"{value:.3f}"
            parts.append(f"{name} min={shown} near={entry['near']} "
                         f"danger={entry['danger']} contact={entry['contact']}")
        self.get_logger().info(
            f"item {record['item']} 结束（{reason}）监视器触发="
            f"{record['monitor_activations']} | " + " | ".join(parts))
        self.in_item = False
        self.cur = None
        self._write()

    def _write(self):
        payload = {
            "args": vars(self.args),
            "boxes": {k: list(v) for k, v in self.boxes.items()},
            "overall": self.overall,
            "items": self.items,
            "monitor": {"activations_total": self.monitor_activations_total,
                        "durations_sec": self.monitor_durations,
                        "events": self.monitor_events[-200:]},
            "samples": self.samples[-200:],
        }
        Path(self.args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(self.args.out).write_text(json.dumps(payload, ensure_ascii=False, indent=1))

    def finish(self):
        if self.in_item:
            self._finish_item(reason="probe shutdown")
        self._write()
        self._print_progress()
        self.get_logger().info(f"结果已写入 {self.args.out}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", default="six_arm")
    parser.add_argument("--boxes", default="moving_box_1,moving_box_2")
    parser.add_argument("--robot-radius", type=float, default=0.30)
    parser.add_argument("--near", type=float, default=0.30, help="接近阈值（m）")
    parser.add_argument("--danger", type=float, default=0.15, help="危险阈值（m）")
    parser.add_argument("--scene", default=DEFAULT_SCENE)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    rclpy.init()
    node = AvoidanceProbe(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
