#!/usr/bin/env python3
"""Plan O / P1（备用）：把移动障碍走廊作为独立 PointCloud2 打进代价地图。

为什么用独立节点：原方案要改 orchestrator 里构建 `/competition/non_target_obstacles` 的 mixin，
但在本会话里定位不到改点（无 grep/glob）。独立节点只需：
  ① 运行本脚本；
  ② 在 nav2 参数的两个 costmap 的 `observation_sources` 里加上 `moving_corridors`（见下）。

nav2 参数（`originbot_nav2_2.yaml`，**local_costmap 与 global_costmap 各一份**）需要加：
```yaml
        observation_sources: scan virtual_cubes moving_corridors
        moving_corridors:
          topic: /competition/moving_corridor_obstacles
          max_obstacle_height: 0.30
          min_obstacle_height: 0.05
          obstacle_max_range: 20.0
          obstacle_min_range: 0.0
          clearing: False
          marking: True
          data_type: "PointCloud2"
```

用法：
  python3 scratch/plan_o_corridor_obstacles.py --scene <world2/obstacles.yaml> [--half 0.45] [--spacing 0.1]

只在 P0 修复后 60 轮仍出现 `contact` > 0 时才启用（一次只改一个变量）。
"""
from pathlib import Path
import argparse
import math

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct

DEFAULT_SCENE = ("/home/ros/桌面/AIC_Competition_Work/runtime/312e163/dev_ws/src/yzbot/"
                 "aic_task/config/scenes/world2/obstacles.yaml")


def corridors_from_scene(path, half_default):
    data = yaml.safe_load(Path(path).read_text()) or {}
    segments = []
    for obstacle in data.get("dynamic_obstacles", []) or []:
        motion = obstacle.get("motion") or {}
        if motion.get("type") != "sine_x":
            continue
        center_x = float(motion["center_x"])
        amplitude = abs(float(motion["amplitude_m"]))
        y = float(motion["center_y"])
        # safety 推导的"宽版"半宽（safety 0.65 + 机器人 0.30 + margin 0.05 = 1.00）仅作参考；
        # 实际生效宽度取 CLI 的 --half（默认 0.45 窄版），否则 max() 会让窄版永远失效。
        safety_half = float(obstacle.get("safety_radius_m", 0.0)) + 0.30 + 0.05
        segments.append((center_x - amplitude, center_x + amplitude, y,
                         float(half_default), safety_half))
    return segments


class CorridorObstacles(Node):
    def __init__(self, args):
        super().__init__("plan_o_corridor_obstacles")
        self.args = args
        self.pub = self.create_publisher(PointCloud2, "/competition/moving_corridor_obstacles", 10)
        self.timer = self.create_timer(0.5, self.publish)
        self.segments = corridors_from_scene(args.scene, args.half)
        self.get_logger().info(
            f"走廊虚拟障碍：{len(self.segments)} 条 生效半宽={args.half}（安全推导宽版见日志下方）"
            f" 间距={args.spacing} 话题=/competition/moving_corridor_obstacles")
        for x_min, x_max, y, half, safety_half in self.segments:
            self.get_logger().info(
                f"  走廊 x∈[{x_min:.3f},{x_max:.3f}] y={y:.3f} 生效半宽={half:.2f} "
                f"(安全推导={safety_half:.2f})")

    def points(self):
        cloud = []
        step = max(0.02, float(self.args.spacing))
        for x_min, x_max, y, half, _safety in self.segments:
            nx = max(2, int(math.ceil((x_max - x_min) / step)) + 1)
            ny = max(2, int(math.ceil((2.0 * half) / step)) + 1)
            for ix in range(nx):
                x = x_min + (x_max - x_min) * ix / (nx - 1)
                for iy in range(ny):
                    yy = y - half + 2.0 * half * iy / (ny - 1)
                    cloud.append((x, yy, float(self.args.z)))
        return cloud

    def publish(self):
        pts = self.points()
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(pts)
        msg.is_dense = True
        msg.data = b"".join(struct.pack("<fff", *p) for p in pts)
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default=DEFAULT_SCENE)
    parser.add_argument("--half", type=float, default=0.45, help="禁止带半宽（窄版默认 0.45 m）")
    parser.add_argument("--spacing", type=float, default=0.10)
    parser.add_argument("--z", type=float, default=0.15)
    args = parser.parse_args()

    rclpy.init()
    node = CorridorObstacles(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
