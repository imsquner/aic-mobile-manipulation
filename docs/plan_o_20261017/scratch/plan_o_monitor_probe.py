#!/usr/bin/env python3
"""Plan O 补充探针：直接测量 collision_monitor 是否真的干预了速度指令。

背景（本次核查发现）：本机 nav2 构建的 collision_monitor **不发布**
`collision_monitor_state`（`/opt/ros/humble/include/nav2_collision_monitor/
collision_monitor_node.hpp` 里既没有 `state_pub_` 也没有 `publishState`），
`ros2 node info /collision_monitor` 的 Publishers 里也没有该话题。
⇒ 原基线里的「碰撞监视器触发 0（从未发布状态）」是**指标不可测**，而不是
「监视器没瞎/瞎了」的证据；`plan_o_avoidance_probe.py` 的 monitor 计数在本构建
下恒为 0，不能用作判据。

本探针改用唯一可靠的观测面：
    /cmd_vel        —— 监视器的输入（velocity_smoother 输出）
    /cmd_vel_safe   —— 监视器的输出（本节点是唯一发布者）
两者一致 ⇒ 监视器未干预；safe 明显小于 in（含归零）⇒ 监视器干预。

只订阅、不发布；除读两个 Twist 话题外对运行中的栈零影响。
"""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def mag(t):
    return math.hypot(t.linear.x, t.angular.z)


class MonitorProbe(Node):
    def __init__(self, out: Path, ratio: float, min_in: float, log_every: float):
        super().__init__("plan_o_monitor_probe")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.out = out
        self.ratio = ratio
        self.min_in = min_in
        self.log_every = log_every

        self.create_subscription(Twist, "/cmd_vel", self.on_in, qos)
        self.create_subscription(Twist, "/cmd_vel_safe", self.on_safe, qos)

        self.t0 = time.monotonic()
        self.last_in = None
        self.last_in_t = 0.0
        self.n_in = 0
        self.n_safe = 0
        self.n_intervene = 0
        self.intervene_first = None
        self.intervene_last = None
        self.prev_intervening = False
        self.events = []
        self.last_report = 0.0
        self.last_write = 0.0
        self.create_timer(1.0, self.tick)
        self.get_logger().info(f"monitor probe 启动 → {out}")

    def on_in(self, msg: Twist):
        self.last_in = msg
        self.last_in_t = time.monotonic()
        self.n_in += 1

    def on_safe(self, msg: Twist):
        self.n_safe += 1
        now = time.monotonic()
        if self.last_in is None or (now - self.last_in_t) > 1.0:
            return
        a = mag(self.last_in)
        b = mag(msg)
        intervening = bool(a > self.min_in and b < self.ratio * a)
        if intervening:
            self.n_intervene += 1
            if self.intervene_first is None:
                self.intervene_first = round(now - self.t0, 2)
            self.intervene_last = round(now - self.t0, 2)
            if not self.prev_intervening and len(self.events) < 500:
                self.events.append({"t": round(now - self.t0, 2),
                                    "in": round(a, 4), "safe": round(b, 4),
                                    "in_x": round(self.last_in.linear.x, 4),
                                    "safe_x": round(msg.linear.x, 4)})
        self.prev_intervening = intervening

    def payload(self):
        return {
            "args": {"ratio": self.ratio, "min_in": self.min_in},
            "elapsed_s": round(time.monotonic() - self.t0, 1),
            "cmd_vel_msgs": self.n_in,
            "cmd_vel_safe_msgs": self.n_safe,
            "interventions": self.n_intervene,
            "intervene_first_s": self.intervene_first,
            "intervene_last_s": self.intervene_last,
            "events": self.events[-200:],
        }

    def write(self):
        self.out.parent.mkdir(parents=True, exist_ok=True)
        self.out.write_text(json.dumps(self.payload(), ensure_ascii=False, indent=1))

    def tick(self):
        now = time.monotonic()
        if now - self.last_write >= 5.0:
            self.last_write = now
            self.write()
        if now - self.t0 - self.last_report >= self.log_every:
            self.last_report = now - self.t0
            p = self.payload()
            self.get_logger().info(
                f"[{p['elapsed_s']:.0f}s] cmd_vel={p['cmd_vel_msgs']} "
                f"cmd_vel_safe={p['cmd_vel_safe_msgs']} "
                f"干预={p['interventions']} "
                f"首次={p['intervene_first_s']} 最近={p['intervene_last_s']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--ratio", type=float, default=0.5,
                    help="safe/in 小于该比例即判为干预")
    ap.add_argument("--min-in", type=float, default=0.05,
                    help="输入速度低于该值时不参与判定")
    ap.add_argument("--log-every", type=float, default=60.0)
    args = ap.parse_args()

    rclpy.init()
    node = MonitorProbe(Path(args.out), args.ratio, args.min_in, args.log_every)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.write()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
