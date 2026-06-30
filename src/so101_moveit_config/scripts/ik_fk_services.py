#!/usr/bin/env python3
"""
Exposes /compute_ik_so and /compute_fk_so as named proxies to MoveIt2's
/compute_ik and /compute_fk services (exact IK via KDL).

Usage:
    ros2 run so101_moveit_config ik_fk_services.py
    (requires move_group to be running)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.srv import GetPositionIK, GetPositionFK


class IKFKServices(Node):
    def __init__(self):
        super().__init__('so101_ik_fk_services')
        cb = ReentrantCallbackGroup()

        self.ik_client = self.create_client(GetPositionIK, '/compute_ik', callback_group=cb)
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk', callback_group=cb)

        self.create_service(GetPositionIK, '/compute_ik_so', self.ik_callback, callback_group=cb)
        self.create_service(GetPositionFK, '/compute_fk_so', self.fk_callback, callback_group=cb)

        self.get_logger().info('Ready: /compute_ik_so  /compute_fk_so')

    async def ik_callback(self, request, response):
        if not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/compute_ik unavailable — is move_group running?')
            return response
        return await self.ik_client.call_async(request)

    async def fk_callback(self, request, response):
        if not self.fk_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/compute_fk unavailable — is move_group running?')
            return response
        return await self.fk_client.call_async(request)


def main():
    rclpy.init()
    node = IKFKServices()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
