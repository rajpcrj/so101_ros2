#!/usr/bin/env python3
"""
Exposes /compute_ik_so_approx and /compute_fk_so_approx as named proxies to
MoveIt2's /compute_ik and /compute_fk services (approximate IK via pick_ik).

Differences from exact version:
  - avoid_collisions forced False  → allows solutions near/in collision
  - timeout bumped to 0.05s        → gives pick_ik room to converge
  - Uses so101_moveit_config_ik_approx's move_group (pick_ik solver)

Usage:
    ros2 run so101_moveit_config_ik_approx ik_fk_services_approx.py
    (requires move_group launched from so101_moveit_config_ik_approx)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from moveit_msgs.srv import GetPositionIK, GetPositionFK


class IKFKServicesApprox(Node):
    def __init__(self):
        super().__init__('so101_ik_fk_services_approx')
        cb = ReentrantCallbackGroup()

        self.ik_client = self.create_client(GetPositionIK, '/compute_ik', callback_group=cb)
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk', callback_group=cb)

        self.create_service(GetPositionIK, '/compute_ik_so_approx', self.ik_callback, callback_group=cb)
        self.create_service(GetPositionFK, '/compute_fk_so_approx', self.fk_callback, callback_group=cb)

        self.get_logger().info('Ready: /compute_ik_so_approx  /compute_fk_so_approx')

    async def ik_callback(self, request, response):
        if not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/compute_ik unavailable — is move_group (approx) running?')
            return response

        # Force approximate-friendly settings
        request.ik_request.avoid_collisions = False
        request.ik_request.timeout = Duration(seconds=0.05).to_msg()

        return await self.ik_client.call_async(request)

    async def fk_callback(self, request, response):
        if not self.fk_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/compute_fk unavailable — is move_group (approx) running?')
            return response
        return await self.fk_client.call_async(request)


def main():
    rclpy.init()
    node = IKFKServicesApprox()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
