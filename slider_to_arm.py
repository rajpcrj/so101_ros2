#!/usr/bin/env python3
"""
slider_to_arm.py — bridge joint_state_publisher_gui sliders to the arm controller.

The demo stack uses ros2_control, whose joint_state_broadcaster OWNS /joint_states.
If the slider GUI also published to /joint_states they would fight. So we run the
GUI publishing to a SIDE topic (/slider_joint_commands) and this node forwards each
slider update to soarmcontroller as a short trajectory — so dragging a slider makes
the real (simulated) arm move, consistent with drag-and-drop and IK execution.

Started automatically by launch_ik_services_v2.sh when sliders are enabled.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']


class SliderToArm(Node):
    def __init__(self):
        super().__init__('slider_to_arm')
        self.client = ActionClient(
            self, FollowJointTrajectory,
            '/soarmcontroller/follow_joint_trajectory')
        self.sub = self.create_subscription(
            JointState, '/slider_joint_commands', self.on_slider, 10)
        self._busy = False
        self._last = None
        self.get_logger().info(
            'Bridging /slider_joint_commands -> soarmcontroller. '
            'Move the GUI sliders to drive the arm.')

    def on_slider(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            target = [name_to_pos[j] for j in ARM_JOINTS]
        except KeyError:
            return  # gripper-only or partial update; ignore
        # Skip if a goal is in flight, to avoid flooding the controller.
        if self._busy:
            self._last = target
            return
        self.send(target)

    def send(self, target):
        if not self.client.server_is_ready():
            return
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in target]
        pt.time_from_start = Duration(sec=0, nanosec=300_000_000)  # 0.3 s
        traj.points = [pt]
        goal.trajectory = traj
        self._busy = True
        fut = self.client.send_goal_async(goal)
        fut.add_done_callback(self._on_sent)

    def _on_sent(self, fut):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self._busy = False
            self._flush()
            return
        res = gh.get_result_async()
        res.add_done_callback(self._on_done)

    def _on_done(self, _):
        self._busy = False
        self._flush()

    def _flush(self):
        if self._last is not None:
            t = self._last
            self._last = None
            self.send(t)


def main():
    rclpy.init()
    node = SliderToArm()
    if not node.client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error(
            'soarmcontroller action server not found — is the demo stack up?')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
