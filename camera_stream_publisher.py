#!/usr/bin/env python3
import os
import yaml
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CAMERA_INFO_PATH = os.path.join(SCRIPT_DIR, 'camera_info.yaml')


class CameraStreamPublisher(Node):
    def __init__(self):
        super().__init__('camera_stream_publisher')

        self.stream_url = os.environ.get('CAMERA_STREAM_URL', 'http://127.0.0.1:15000/stream')
        self.camera_name = os.environ.get('CAMERA_NAME', 'camera')

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.camera_info_msg = self._load_camera_info()

        self.cap = None
        self._connect()

        self.create_timer(1.0 / 30.0, self._publish_frame)
        self.get_logger().info(f'Publishing {self.camera_name} stream from {self.stream_url}')

    def _load_camera_info(self):
        msg = CameraInfo()
        msg.header.frame_id = 'camera_optical_frame'
        try:
            with open(CAMERA_INFO_PATH, 'r') as f:
                d = yaml.safe_load(f)
            msg.width = d['image_width']
            msg.height = d['image_height']
            msg.distortion_model = d['distortion_model']
            msg.d = d['distortion_coefficients']['data']
            msg.k = d['camera_matrix']['data']
            msg.r = d['rectification_matrix']['data']
            msg.p = d['projection_matrix']['data']
        except Exception as e:
            self.get_logger().warn(f'camera_info.yaml load failed: {e}')
        return msg

    def _connect(self):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(self.stream_url)
        if self.cap.isOpened():
            self.get_logger().info(f'Connected to {self.stream_url}')
        else:
            self.get_logger().error(f'Could not open stream: {self.stream_url} — retrying on next frame')

    def _publish_frame(self):
        if not self.cap.isOpened():
            self._connect()
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Stream read failed — reconnecting...')
            self._connect()
            return

        now = self.get_clock().now().to_msg()

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = now
        img_msg.header.frame_id = 'camera_optical_frame'
        self.image_pub.publish(img_msg)

        self.camera_info_msg.header.stamp = now
        self.info_pub.publish(self.camera_info_msg)


def main():
    rclpy.init()
    node = CameraStreamPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
