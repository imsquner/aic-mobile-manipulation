#!/usr/bin/env python3
"""
Nav2 Lifecycle Activator Node.
Waits for Nav2 lifecycle services and activates any nodes in 'inactive' state.
Runs once, then exits. Designed to be launched as part of the competition bringup.
"""
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
import time


class Nav2LifecycleActivator(Node):
    def __init__(self):
        super().__init__('nav2_lifecycle_activator')
        self.nodes_to_activate = [
            'map_server', 'amcl', 'controller_server', 'planner_server',
            'bt_navigator', 'behavior_server', 'velocity_smoother', 'waypoint_follower',
        ]
        self.active_count = 0
        self.timer = self.create_timer(5.0, self._try_activate)
        self.attempt = 0
        self.get_logger().info('Nav2 Lifecycle Activator started')

    def _try_activate(self):
        self.attempt += 1
        self.get_logger().info(f'Activation attempt {self.attempt}...')
        self.active_count = 0

        for name in self.nodes_to_activate:
            # First check current state
            get_svc = '/' + name + '/get_state'
            get_cli = self.create_client(GetState, get_svc)

            if not get_cli.wait_for_service(timeout_sec=3.0):
                self.get_logger().debug(f'/{name}: get_state not available')
                self.destroy_client(get_cli)
                continue

            req = GetState.Request()
            future = get_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            state_id = -1
            if future.result():
                state_id = future.result().current_state.id
            self.destroy_client(get_cli)

            if state_id == 3:  # Already active
                self.get_logger().info(f'  /{name}: active')
                self.active_count += 1
                continue
            elif state_id == 4:  # Finalized - skip
                self.get_logger().warn(f'  /{name}: finalized, skipping')
                continue
            elif state_id not in (1, 2):  # Need unconfigured or inactive
                self.get_logger().warn(f'  /{name}: state={state_id}, skipping')
                continue

            # Activate (for inactive) or configure+activate (for unconfigured)
            chg_svc = '/' + name + '/change_state'
            chg_cli = self.create_client(ChangeState, chg_svc)

            if not chg_cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(f'  /{name}: change_state not available')
                self.destroy_client(chg_cli)
                continue

            success = True

            # Configure if unconfigured
            if state_id == 1:
                req2 = ChangeState.Request()
                req2.transition.id = Transition.TRANSITION_CONFIGURE
                f2 = chg_cli.call_async(req2)
                rclpy.spin_until_future_complete(self, f2, timeout_sec=10.0)
                if not f2.result() or not f2.result().success:
                    self.get_logger().warn(f'  /{name}: configure failed')
                    self.destroy_client(chg_cli)
                    continue
                self.get_logger().info(f'  /{name}: configured')

            # Activate
            req3 = ChangeState.Request()
            req3.transition.id = Transition.TRANSITION_ACTIVATE
            f3 = chg_cli.call_async(req3)
            rclpy.spin_until_future_complete(self, f3, timeout_sec=10.0)
            if f3.result() and f3.result().success:
                self.get_logger().info(f'  /{name}: activated')
                self.active_count += 1
            else:
                self.get_logger().warn(f'  /{name}: activate failed')
                success = False

            self.destroy_client(chg_cli)

        self.get_logger().info(f'Active: {self.active_count}/{len(self.nodes_to_activate)}')

        if self.active_count >= 6 or self.attempt >= 6:
            self.get_logger().info(
                f'Activation complete: {self.active_count}/{len(self.nodes_to_activate)} active'
            )
            self.timer.cancel()
            # Don't shutdown — let the node stay alive so start_competition can track it


def main():
    rclpy.init()
    node = Nav2LifecycleActivator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

