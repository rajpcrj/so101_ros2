#!/usr/bin/env python3
"""
move_to_ik.py — close the loop: pose -> IK -> MOVE the arm in RViz.

This is the "see it move" companion to ik_fk_services.py. You give it either:
  * a Cartesian POSE (x y z [qx qy qz qw])  -> it calls /compute_ik_so to get
    joint angles, then executes them, or
  * a JOINT goal (5 angles, radians)         -> it executes them directly.

Either way it sends a FollowJointTrajectory goal to `soarmcontroller`
(the trajectory controller spawned by demo.launch.py), so the arm visibly
moves to the target in RViz.

Requires the v2 stack to be running (launch_ik_services_v2.sh):
  - move_group + ros2_control (soarmcontroller / follow_joint_trajectory)
  - /compute_ik_so  (only needed for --pose mode)

EXAMPLES
  # Move to a joint configuration (always works, great first test):
  python3 move_to_ik.py --joints 0.3 0.2 0.1 0.0 0.0

  # Move to a Cartesian pose (position only; orientation defaults to identity):
  python3 move_to_ik.py --pose 0.2 0.0 0.15

  # Move to a full pose (position + quaternion):
  python3 move_to_ik.py --pose 0.2 0.0 0.15 0.0 0.0 0.0 1.0

TIP: to get a pose you KNOW is reachable, pick joints, run FK on them first:
  ros2 service call /compute_fk_so moveit_msgs/srv/GetPositionFK \
    "{header: {frame_id: 'base_link'}, fk_link_names: ['wrist_link'], \
      robot_state: {joint_state: {name: ['shoulder_pan','shoulder_lift', \
      'elbow_flex','wrist_flex','wrist_roll'], position: [0.3,0.2,0.1,0,0]}}}"
  ...then feed that pose into --pose.
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
GROUP = 'soArm'
BASE_FRAME = 'base_link'
# The soArm group's real tip link (per the SRDF chain). MoveIt reports
# "Available tip frames: [gripper_link, ]" — NOT wrist_link. FK/IK must use this.
TIP_LINK = 'gripper_link'
# A neutral seed so pick_ik has a starting state ("Found empty JointState
# message" -> instant -31 otherwise). Order matches ARM_JOINTS.
SEED_POSITIONS = [0.0, 0.0, 0.0, 0.0, 0.0]

# MoveIt error codes we care about (moveit_msgs/MoveItErrorCodes)
ERR = {1: 'SUCCESS', -15: 'INVALID_GROUP_NAME', -31: 'NO_IK_SOLUTION'}


class MoveToIK(Node):
    def __init__(self, ik_service='/compute_ik_so'):
        super().__init__('move_to_ik')
        self.ik_service = ik_service
        self.ik_client = self.create_client(GetPositionIK, ik_service)
        self.traj_client = ActionClient(
            self, FollowJointTrajectory,
            '/soarmcontroller/follow_joint_trajectory')

    def solve_ik(self, pose_xyzq):
        """pose_xyzq = [x,y,z,qx,qy,qz,qw] -> list of 5 joint angles, or None."""
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f'{self.ik_service} not available — is the matching stack running?')
            return None

        req = GetPositionIK.Request()
        ikr = PositionIKRequest()
        ikr.group_name = GROUP
        # Leave ik_link_name empty: MoveIt uses the group's default tip
        # (wrist_link). Setting it explicitly can make MoveIt treat the pose's
        # reference frame as the tip link itself -> "Cannot compute IK ... pose
        # reference frame 'wrist_link'" and an instant failure.
        ikr.avoid_collisions = True
        ps = PoseStamped()
        ps.header.frame_id = BASE_FRAME
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pose_xyzq[0:3]
        ps.pose.orientation.x, ps.pose.orientation.y, \
            ps.pose.orientation.z, ps.pose.orientation.w = pose_xyzq[3:7]
        ikr.pose_stamped = ps
        ikr.timeout = Duration(sec=2)
        # Seed the solver with a non-empty start state (required by pick_ik).
        seed = JointState()
        seed.name = list(ARM_JOINTS)
        seed.position = list(SEED_POSITIONS)
        ikr.robot_state.joint_state = seed
        req.ik_request = ikr

        self.get_logger().info(f'Calling IK for pose {pose_xyzq} ...')
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        resp = future.result()
        if resp is None:
            self.get_logger().error('IK call failed (no response).')
            return None

        code = resp.error_code.val
        if code != 1:
            self.get_logger().error(
                f'IK failed: error_code={code} ({ERR.get(code, "see MoveItErrorCodes")}). '
                'This 5-DOF arm often cannot reach an arbitrary 6-DOF pose — '
                'try a pose obtained from FK, or use the approx IK service.')
            return None

        # Pull out the 5 arm joints in the order the controller expects.
        names = list(resp.solution.joint_state.name)
        positions = list(resp.solution.joint_state.position)
        name_to_pos = dict(zip(names, positions))
        try:
            joints = [name_to_pos[j] for j in ARM_JOINTS]
        except KeyError as e:
            self.get_logger().error(f'IK solution missing joint {e}; got {names}')
            return None
        self.get_logger().info(
            'IK solution: ' +
            ', '.join(f'{j}={v:+.4f}' for j, v in zip(ARM_JOINTS, joints)))
        return joints

    def move_to_joints(self, joints, seconds=3.0):
        """Send a single-point trajectory to the arm controller and wait."""
        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                '/soarmcontroller/follow_joint_trajectory not available — '
                'is move_group / ros2_control running (demo stack)?')
            return False

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in joints]
        sec = int(seconds)
        pt.time_from_start = Duration(sec=sec, nanosec=int((seconds - sec) * 1e9))
        traj.points = [pt]
        goal.trajectory = traj

        self.get_logger().info(f'Executing move over {seconds:.1f}s ...')
        send_future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('Trajectory goal rejected.')
            return False

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=seconds + 10.0)
        result = result_future.result()
        if result is None:
            self.get_logger().error('No result from trajectory execution.')
            return False
        self.get_logger().info('Move complete. Arm should now be at the target in RViz.')
        return True


def main():
    p = argparse.ArgumentParser(
        description='Pose -> IK -> move the SO-101 arm in RViz.')
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument('--pose', nargs='+', type=float, metavar='V',
                   help='x y z [qx qy qz qw] in base_link frame')
    g.add_argument('--joints', nargs=5, type=float, metavar='J',
                   help='5 joint angles (rad): shoulder_pan shoulder_lift '
                        'elbow_flex wrist_flex wrist_roll')
    p.add_argument('--time', type=float, default=3.0,
                   help='seconds to take for the move (default 3.0)')
    p.add_argument('--approx', action='store_true',
                   help='use /compute_ik_so_approx (pick_ik, tolerant) instead of '
                        'the exact KDL /compute_ik_so. Requires the APPROX move_group '
                        'stack to be running.')
    args = p.parse_args()

    ik_service = '/compute_ik_so_approx' if args.approx else '/compute_ik_so'
    rclpy.init()
    node = MoveToIK(ik_service=ik_service)
    ok = False
    try:
        if args.joints is not None:
            ok = node.move_to_joints(args.joints, args.time)
        else:
            pose = list(args.pose)
            if len(pose) == 3:
                pose += [0.0, 0.0, 0.0, 1.0]  # identity orientation
            elif len(pose) != 7:
                node.get_logger().error(
                    '--pose needs 3 (x y z) or 7 (x y z qx qy qz qw) values.')
                sys.exit(2)
            joints = node.solve_ik(pose)
            if joints is not None:
                ok = node.move_to_joints(joints, args.time)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
