"""World2 robot, scan, and model-state snapshots."""
import math, time
import rclpy
from .timing import evaluate_transform_age

class ZoneSlotSensingMixin:
    'Sensor and model-state reads'

    def _gazebo_robot_pose_map(self):
        """机器人 (x, y, yaw)：直接取自 Gazebo ModelStates 的 world 坐标。

        本场景 world 与 map 同坐标系（见 _slot_current_model_positions_map 的说明），
        所以 Gazebo 真值可以直接当 map 位姿用。相比原先查 `map->odom`(AMCL ~2-3 Hz)
        与 `odom->base`(Gazebo diff_drive) 两条 TF，这条路不依赖任何低频 TF，在高负载
        下也不会因为 0.4-1.1 s 的更新空档而把合法数据判成过期。
        """
        states = self._latest_model_states
        if states is None:
            return None
        try:
            robot_name = str(self.get_parameter("robot_model_name").value)
        except Exception:
            return None
        try:
            index = states.name.index(robot_name)
        except ValueError:
            return None
        pose = states.pose[index]
        x = float(pose.position.x)
        y = float(pose.position.y)
        orientation = pose.orientation
        qx, qy, qz, qw = (
            float(orientation.x), float(orientation.y),
            float(orientation.z), float(orientation.w),
        )
        if not all(math.isfinite(value) for value in (x, y, qx, qy, qz, qw)):
            return None
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1.0e-6:
            return None
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        return x, y, yaw

    def _slot_scan_points_map(self):
        """把 LaserScan 转到 map 系：Gazebo 机器人位姿 + 静态 base->laser 变换。

        与 _slot_current_model_positions_map 同理，world 与 map 同坐标系，机器人在
        map 的位姿可直接取 Gazebo ModelStates。原先查 `map->laser_link` 需要经过
        `map->odom`(AMCL，~2-3 Hz) 与 `odom->base`(Gazebo diff_drive) 两段低频 TF，
        其"最新时间戳"只能跟着最慢的一段刷新；2026-09-26 实测精确查询抛
        ExtrapolationException、回退后又因 transform_age 0.405 s > 0.35 s 被判
        "transform_stale"，槽位机动在抓取阶段即被拦下。

        这里改用"Gazebo 真值位姿 + base_footprint->laser_link 静态变换"，完全不依赖
        低频 TF。保留的新鲜度门只有：LaserScan 的墙钟接收时间、ROS 时间戳，以及扫描
        本身的帧/角度/距离合法性。
        """
        scan = self._latest_scan
        max_age = float(
            self.points.get("approach", {}).get("slot_scan_max_age_sec", 1.0)
        )
        if not math.isfinite(max_age) or max_age <= 0.0:
            return None, "LaserScan freshness limit is invalid"
        if scan is None or time.monotonic() - self._latest_scan_received_at > max_age:
            return None, "LaserScan missing or stale"
        stamp = scan.header.stamp
        scan_stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        now = self.get_clock().now()
        scan_fresh, _, scan_age_reason = evaluate_transform_age(
            int(now.nanoseconds),
            scan_stamp_ns,
            max_age,
        )
        if not scan_fresh:
            return None, f"LaserScan ROS timestamp is invalid or stale: {scan_age_reason}"
        if not scan.header.frame_id or len(scan.ranges) < 180:
            return None, "LaserScan frame or angular sample count is invalid"
        angle_min = float(scan.angle_min)
        angle_max = float(scan.angle_max)
        angle_increment = float(scan.angle_increment)
        angle_span = angle_max - angle_min
        expected_span = (len(scan.ranges) - 1) * angle_increment
        range_min = float(scan.range_min)
        range_max = float(scan.range_max)
        if (
            not math.isfinite(angle_min)
            or not math.isfinite(angle_max)
            or not math.isfinite(angle_increment)
            or angle_increment <= 0.0
            or angle_span <= 0.0
            or abs(angle_span - expected_span)
            > max(1.0e-4, 0.51 * angle_increment)
            or angle_span + angle_increment
            < 2.0 * math.pi - 2.0 * angle_increment
            or not math.isfinite(range_min)
            or range_min <= 0.0
            or not math.isfinite(range_max)
            or range_max <= range_min
        ):
            return None, "LaserScan does not provide a valid full angular sweep"
        robot_pose = self._gazebo_robot_pose_map()
        if robot_pose is None:
            return None, "Gazebo robot pose unavailable for LaserScan transform"
        base_x, base_y, base_yaw = robot_pose
        # base_footprint -> laser_link 是静态变换（/tf_static），不会过期。
        try:
            base_to_laser = self._tf_buffer.lookup_transform(
                "base_footprint",
                scan.header.frame_id,
                rclpy.time.Time(),
            )
        except Exception:
            return None, "base-to-laser static transform unavailable"
        lt = base_to_laser.transform.translation
        lr = base_to_laser.transform.rotation
        lx_off = float(lt.x)
        ly_off = float(lt.y)
        lqx, lqy, lqz, lqw = (
            float(lr.x), float(lr.y), float(lr.z), float(lr.w),
        )
        if not all(math.isfinite(value) for value in (
            base_x, base_y, base_yaw, lx_off, ly_off, lqx, lqy, lqz, lqw,
        )):
            return None, "LaserScan transform inputs are non-finite"
        lnorm = math.sqrt(lqx * lqx + lqy * lqy + lqz * lqz + lqw * lqw)
        if lnorm < 1.0e-6:
            return None, "base-to-laser transform has invalid orientation"
        lqx, lqy, lqz, lqw = lqx / lnorm, lqy / lnorm, lqz / lnorm, lqw / lnorm
        laser_yaw = math.atan2(
            2.0 * (lqw * lqz + lqx * lqy),
            1.0 - 2.0 * (lqy * lqy + lqz * lqz),
        )
        cos_l, sin_l = math.cos(laser_yaw), math.sin(laser_yaw)
        cos_b, sin_b = math.cos(base_yaw), math.sin(base_yaw)
        points = []
        angle = angle_min
        for raw_range in scan.ranges:
            distance = float(raw_range)
            if math.isnan(distance) or distance < range_min:
                return None, "LaserScan contains an invalid or blind-range return"
            if math.isinf(distance):
                if distance < 0.0:
                    return None, "LaserScan contains a negative infinite return"
                angle += angle_increment
                continue
            if distance > range_max:
                angle += angle_increment
                continue
            laser_x = distance * math.cos(angle)
            laser_y = distance * math.sin(angle)
            # laser_link -> base_footprint（静态）
            body_x = lx_off + cos_l * laser_x - sin_l * laser_y
            body_y = ly_off + sin_l * laser_x + cos_l * laser_y
            # base_footprint -> map（Gazebo 真值位姿）
            map_x = base_x + cos_b * body_x - sin_b * body_y
            map_y = base_y + sin_b * body_x + cos_b * body_y
            if not math.isfinite(map_x) or not math.isfinite(map_y):
                return None, "LaserScan transform produced a non-finite point"
            points.append((map_x, map_y))
            angle += angle_increment
        if not points and not all(
            (math.isinf(float(raw_range)) and float(raw_range) > 0.0)
            or float(raw_range) > range_max
            for raw_range in scan.ranges
        ):
            return None, "LaserScan has no usable returns and is not explicitly clear"
        return points, ""

    def _slot_current_model_positions_map(self):
        """把 Gazebo ModelStates 直接当作 map 坐标系下的快照。

        本场景的 Gazebo world 与 map 是同一坐标系：points 的 robot_initial_pose
        同时用于 Gazebo spawn（world 系）与 AMCL 初始化（map 系），而且物块坐标
        直接查 mapn3.pgm 得到的净空值合理（red_cube_1 0.447 m、zone_A 0.212 m 等）。

        因此原先"用机器人 TF 位姿构造 world->map 刚体变换"这一整条链路 —— 以及
        为它服务的 map->base / map->odom / odom->base 三道 TF 新鲜度检查 —— 都是
        冗余的。取消它们的收益有两层：
          1. 去掉对 AMCL（~2-3 Hz）与 Gazebo diff_drive TF 的强依赖。这两者在本 VM
             高负载下会出现 0.4-1.1 s 的更新空档，正是槽位流程被反复误拦的根源；
          2. 去掉每条判据的重复 TF 查询，直接降低编排器自身 CPU 占用。

        保留的唯一门是 ModelStates 的**接收**新鲜度（墙钟）与数值有限性。
        """
        states = self._latest_model_states
        max_age = float(
            self.points.get("approach", {}).get(
                "slot_model_states_max_age_sec", 1.0
            )
        )
        if not math.isfinite(max_age) or max_age <= 0.0:
            return None, "Gazebo ModelStates freshness limit is invalid"
        if (
            states is None
            or time.monotonic() - self._latest_model_states_received_at > max_age
        ):
            return None, "Gazebo ModelStates missing or stale"
        if len(states.name) != len(states.pose) or len(set(states.name)) != len(states.name):
            return None, "Gazebo ModelStates names and poses are inconsistent"
        positions = {}
        for name, pose in zip(states.name, states.pose):
            world_x = float(pose.position.x)
            world_y = float(pose.position.y)
            world_z = float(pose.position.z)
            orientation = pose.orientation
            oqx, oqy, oqz, oqw = (
                float(orientation.x), float(orientation.y),
                float(orientation.z), float(orientation.w),
            )
            if not all(math.isfinite(value) for value in (
                world_x, world_y, world_z, oqx, oqy, oqz, oqw,
            )):
                continue
            orientation_norm = math.sqrt(
                oqx * oqx + oqy * oqy + oqz * oqz + oqw * oqw
            )
            if orientation_norm < 1.0e-6:
                continue
            positions[str(name)] = (world_x, world_y, world_z)
        return positions, ""
