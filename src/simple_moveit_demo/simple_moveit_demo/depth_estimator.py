#!/usr/bin/env python3
"""
Depth estimator — projects YOLO bounding boxes to 3D using the top-down
camera's known geometry (no depth sensor required).

Pipeline:
  /camera/image_raw  ──► YOLOv8n ──► bbox center ──► pinhole projection
                                                             │
                                              /object_positions (PointStamped)

Note: YOLOv8n is pretrained on COCO. Expect 0 detections on synthetic top-down
shapes — this is the documented domain gap. See README → Domain Gap Analysis.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

from simple_moveit_demo.shape_classifier import YOLODetector

# Top-down camera pose in panda_link0 frame
CAM_X, CAM_Y, CAM_H = 0.40, 0.00, 0.80   # metres


def _pixel_to_robot(u: float, v: float,
                    fx: float, fy: float,
                    cx: float, cy: float) -> tuple:
    """Project bbox centre to robot floor-plane (z=0) via pinhole model."""
    wy = CAM_Y + (u - cx) * CAM_H / fx
    wx = CAM_X - (v - cy) * CAM_H / fy
    return wx, wy, 0.0


class DepthEstimator(Node):
    def __init__(self):
        super().__init__('depth_estimator')

        self._bridge = CvBridge()
        self._yolo   = YOLODetector()
        self._fx = self._fy = 520.0
        self._cx, self._cy = 320.0, 240.0

        self._pub = self.create_publisher(PointStamped, '/object_positions', 10)

        self.create_subscription(CameraInfo, '/camera/camera_info',
                                 self._camera_info_cb, 10)
        self.create_subscription(Image, '/camera/image_raw',
                                 self._image_cb, 10)

        self.get_logger().info(
            'Depth estimator ready — YOLOv8n bbox → 3D position on /object_positions')

    def _camera_info_cb(self, msg: CameraInfo):
        K = msg.k
        if len(K) == 9 and K[0] > 0:
            self._fx, self._fy = K[0], K[4]
            self._cx, self._cy = K[2], K[5]

    def _image_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        detections = self._yolo.detect(frame)

        if not detections:
            self.get_logger().debug(
                'No YOLO detections — domain gap: COCO weights vs synthetic top-down camera')
            return

        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            u_c = (x1 + x2) / 2.0
            v_c = (y1 + y2) / 2.0
            wx, wy, wz = _pixel_to_robot(
                u_c, v_c, self._fx, self._fy, self._cx, self._cy)

            pt = PointStamped()
            pt.header.stamp    = msg.header.stamp
            pt.header.frame_id = 'panda_link0'
            pt.point.x = wx
            pt.point.y = wy
            pt.point.z = wz
            self._pub.publish(pt)

            self.get_logger().info(
                f"YOLO [{det['class']} {det['confidence']:.2f}] → "
                f"3D ({wx:.3f}, {wy:.3f}, {wz:.3f}) on /object_positions")


def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
