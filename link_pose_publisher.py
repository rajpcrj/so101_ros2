#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, TransformStamped

# Camera mount offset from gripper_link (from Isaac Sim so_101_camera transform)
CAMERA_TRANSLATION = (-0.00195, 0.04806, -0.06005)  # metres
CAMERA_EULER_DEG   = (45.0, 0.0, -90.0)             # XYZ extrinsic, degrees


def euler_xyz_to_quat(rx_deg, ry_deg, rz_deg):
    """XYZ extrinsic euler angles (degrees) → (qx, qy, qz, qw)."""
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)
    cx, sx = math.cos(rx / 2), math.sin(rx / 2)
    cy, sy = math.cos(ry / 2), math.sin(ry / 2)
    cz, sz = math.cos(rz / 2), math.sin(rz / 2)
    # q = Rz * Ry * Rx
    qw = cx*cy*cz + sx*sy*sz
    qx = sx*cy*cz - cx*sy*sz
    qy = cx*sy*cz + sx*cy*sz
    qz = cx*cy*sz - sx*sy*cz
    return qx, qy, qz, qw

LINKS = [
    'base_link',
    'shoulder_link',
    'upper_arm_link',
    'lower_arm_link',
    'wrist_link',
    'gripper_link',
    'gripper_frame_link',
    'moving_jaw_so101_v1_link',
]

# One distinct color per link (RGB 0-1)
COLORS = [
    (1.0, 0.2, 0.2),  # red        – base
    (1.0, 0.6, 0.0),  # orange     – shoulder
    (1.0, 1.0, 0.0),  # yellow     – upper_arm
    (0.2, 1.0, 0.2),  # green      – lower_arm
    (0.0, 1.0, 0.8),  # cyan       – wrist
    (0.2, 0.4, 1.0),  # blue       – gripper
    (0.7, 0.0, 1.0),  # violet     – gripper_frame
    (1.0, 0.0, 0.6),  # magenta    – moving_jaw
]


class LinkPosePublisher(Node):
    def __init__(self):
        super().__init__('link_pose_publisher')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, '/link_poses/markers', 10)

        # Individual PoseStamped topic per link
        self.pose_pubs = {
            link: self.create_publisher(PoseStamped, f'/link_poses/{link}', 10)
            for link in LINKS
        }

        # Broadcast so_101_camera as a static TF frame off gripper_link
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_camera_tf()

        self.timer = self.create_timer(0.1, self.publish_poses)
        self.get_logger().info('Link pose publisher running — publishing to /link_poses/markers')

    def _publish_camera_tf(self):
        qx, qy, qz, qw = euler_xyz_to_quat(*CAMERA_EULER_DEG)
        tx, ty, tz = CAMERA_TRANSLATION

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'gripper_link'
        tf_msg.child_frame_id = 'so_101_camera'
        tf_msg.transform.translation.x = tx
        tf_msg.transform.translation.y = ty
        tf_msg.transform.translation.z = tz
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self._static_broadcaster.sendTransform(tf_msg)

    def publish_poses(self):
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        for i, link in enumerate(LINKS):
            try:
                t = self.tf_buffer.lookup_transform('base_link', link, Time())
            except Exception:
                continue

            pos = t.transform.translation
            rot = t.transform.rotation
            r, g, b = COLORS[i % len(COLORS)]

            # Publish individual PoseStamped
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = 'base_link'
            ps.pose.position.x = pos.x
            ps.pose.position.y = pos.y
            ps.pose.position.z = pos.z
            ps.pose.orientation = rot
            self.pose_pubs[link].publish(ps)

            # Arrow marker (shows frame x-axis direction)
            arrow = Marker()
            arrow.header.stamp = now
            arrow.header.frame_id = 'base_link'
            arrow.ns = 'arrows'
            arrow.id = i * 2
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = pos.x
            arrow.pose.position.y = pos.y
            arrow.pose.position.z = pos.z
            arrow.pose.orientation = rot
            arrow.scale.x = 0.07   # arrow length
            arrow.scale.y = 0.008  # shaft diameter
            arrow.scale.z = 0.012  # head diameter
            arrow.color.r = r
            arrow.color.g = g
            arrow.color.b = b
            arrow.color.a = 1.0
            markers.markers.append(arrow)

            # Text label floating above the arrow
            label = Marker()
            label.header.stamp = now
            label.header.frame_id = 'base_link'
            label.ns = 'labels'
            label.id = i * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = pos.x
            label.pose.position.y = pos.y
            label.pose.position.z = pos.z + 0.04
            label.pose.orientation.w = 1.0
            label.scale.z = 0.018
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = link
            markers.markers.append(label)

        # Camera marker — sphere + label at so_101_camera frame
        try:
            ct = self.tf_buffer.lookup_transform('base_link', 'so_101_camera', Time())
            cp = ct.transform.translation
            cr = ct.transform.rotation
            base_id = len(LINKS) * 2

            sphere = Marker()
            sphere.header.stamp = now
            sphere.header.frame_id = 'base_link'
            sphere.ns = 'camera'
            sphere.id = base_id
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = cp.x
            sphere.pose.position.y = cp.y
            sphere.pose.position.z = cp.z
            sphere.pose.orientation = cr
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.025
            sphere.color.r = 1.0
            sphere.color.g = 1.0
            sphere.color.b = 0.0
            sphere.color.a = 1.0
            markers.markers.append(sphere)

            cam_arrow = Marker()
            cam_arrow.header.stamp = now
            cam_arrow.header.frame_id = 'base_link'
            cam_arrow.ns = 'camera'
            cam_arrow.id = base_id + 1
            cam_arrow.type = Marker.ARROW
            cam_arrow.action = Marker.ADD
            cam_arrow.pose.position.x = cp.x
            cam_arrow.pose.position.y = cp.y
            cam_arrow.pose.position.z = cp.z
            cam_arrow.pose.orientation = cr
            cam_arrow.scale.x = 0.07
            cam_arrow.scale.y = 0.010
            cam_arrow.scale.z = 0.014
            cam_arrow.color.r = 1.0
            cam_arrow.color.g = 1.0
            cam_arrow.color.b = 0.0
            cam_arrow.color.a = 1.0
            markers.markers.append(cam_arrow)

            cam_label = Marker()
            cam_label.header.stamp = now
            cam_label.header.frame_id = 'base_link'
            cam_label.ns = 'camera'
            cam_label.id = base_id + 2
            cam_label.type = Marker.TEXT_VIEW_FACING
            cam_label.action = Marker.ADD
            cam_label.pose.position.x = cp.x
            cam_label.pose.position.y = cp.y
            cam_label.pose.position.z = cp.z + 0.04
            cam_label.pose.orientation.w = 1.0
            cam_label.scale.z = 0.018
            cam_label.color.r = 1.0
            cam_label.color.g = 1.0
            cam_label.color.b = 0.0
            cam_label.color.a = 1.0
            cam_label.text = 'so_101_camera'
            markers.markers.append(cam_label)
        except Exception:
            pass

        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = LinkPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
