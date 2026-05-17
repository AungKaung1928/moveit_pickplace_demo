#!/usr/bin/env python3
"""
Synthetic top-down camera: one red ball, one orange box outline.
Ball disappears from image when picked up (via /scene_object_updates).
"""

import math
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

IMG_W, IMG_H = 640, 480
FX = FY = 520.0
CX, CY = IMG_W / 2.0, IMG_H / 2.0

CAM_X, CAM_Y, CAM_H = 0.40, 0.00, 0.80

BALL_WX, BALL_WY = 0.45, 0.0
BOX_WX,  BOX_WY  = 0.55, 0.25


def world_to_pixel(wx: float, wy: float) -> tuple[int, int]:
    u = int(CX + FX * (wy - CAM_Y) / CAM_H)
    v = int(CY - FY * (wx - CAM_X) / CAM_H)
    return u, v


def meters_to_pixels(m: float) -> int:
    return max(int(FX * m / CAM_H), 4)


class CameraSimulator(Node):
    def __init__(self):
        super().__init__('camera_simulator')
        self.declare_parameter('publish_rate', 10.0)
        rate = self.get_parameter('publish_rate').value

        self._bridge = CvBridge()
        self._frame  = 0
        self._ball_hidden = False
        self._lock = threading.Lock()

        self.create_subscription(String, '/scene_object_updates',
                                 self._update_cb, 10)

        self._img_pub  = self.create_publisher(Image,      '/camera/image_raw',   10)
        self._info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(f'Camera simulator: {IMG_W}x{IMG_H} @ {rate:.0f} Hz')

    def _update_cb(self, msg: String) -> None:
        parts = msg.data.split(':')
        if len(parts) != 2:
            return
        cmd, obj_id = parts
        with self._lock:
            if obj_id == 'ball':
                self._ball_hidden = (cmd == 'remove')

    def _build_image(self) -> np.ndarray:
        img = np.full((IMG_H, IMG_W, 3), 40, dtype=np.uint8)

        # Grid lines
        for gx in range(0, IMG_W, 40):
            cv2.line(img, (gx, 0), (gx, IMG_H), (55, 55, 55), 1)
        for gy in range(0, IMG_H, 40):
            cv2.line(img, (0, gy), (IMG_W, gy), (55, 55, 55), 1)

        # Orange box outline
        bu, bv = world_to_pixel(BOX_WX, BOX_WY)
        half_px = meters_to_pixels(0.07)
        cv2.rectangle(img, (bu - half_px, bv - half_px),
                      (bu + half_px, bv + half_px), (0, 140, 255), 3)
        cv2.putText(img, 'BOX', (bu - half_px, bv - half_px - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 160, 255), 2)

        # Red ball (disappears when picked)
        with self._lock:
            hidden = self._ball_hidden

        if not hidden:
            t = self._frame * 0.05
            wx = BALL_WX + 0.003 * math.sin(t)
            wy = BALL_WY + 0.003 * math.cos(t * 1.3)
            cu, cv_ = world_to_pixel(wx, wy)
            r_px = meters_to_pixels(0.030)
            cv2.circle(img, (cu, cv_), r_px, (0, 20, 220), -1)        # red fill
            cv2.circle(img, (cu, cv_), r_px, (255, 255, 255), 2)      # white outline
            cv2.putText(img, 'BALL', (cu - 18, cv_ + r_px + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (220, 220, 220), 1)

        self._frame += 1
        return img

    def _publish(self):
        now = self.get_clock().now().to_msg()
        img = self._build_image()

        img_msg = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_msg.header.stamp = now
        img_msg.header.frame_id = 'camera_link'
        self._img_pub.publish(img_msg)

        info = CameraInfo()
        info.header.stamp = now
        info.header.frame_id = 'camera_link'
        info.width  = IMG_W
        info.height = IMG_H
        info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.distortion_model = 'plumb_bob'
        self._info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = CameraSimulator()
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
