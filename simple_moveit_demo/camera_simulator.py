#!/usr/bin/env python3
"""
Synthetic top-down camera that publishes ROS Image + CameraInfo.
Draws colored shapes (with ArUco markers overlaid) on a simulated table.
No physical camera needed — run this to test the full vision pipeline.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

# Camera intrinsics (640×480, simulated)
IMG_W, IMG_H = 640, 480
FX = FY = 520.0
CX, CY = IMG_W / 2.0, IMG_H / 2.0

# Camera mounted at (0.40, 0.0, 0.80) in panda_link0, looking straight down
CAM_X, CAM_Y, CAM_H = 0.40, 0.00, 0.80

COLORS_BGR = {
    'red':    (0,   30,  220),
    'green':  (30,  180,  30),
    'blue':   (200,  60,  20),
    'yellow': (0,   200, 220),
}

# Objects in the scene: each has world (x, y) and a colour + shape
SCENE_OBJECTS = [
    {'id': 0, 'color': 'red',    'shape': 'rect',   'wx': 0.42, 'wy':  0.12, 'size': 0.06},
    {'id': 1, 'color': 'green',  'shape': 'circle', 'wx': 0.50, 'wy': -0.10, 'size': 0.055},
    {'id': 2, 'color': 'blue',   'shape': 'rect',   'wx': 0.33, 'wy': -0.18, 'size': 0.065},
    {'id': 3, 'color': 'yellow', 'shape': 'circle', 'wx': 0.48, 'wy':  0.20, 'size': 0.05},
]


def world_to_pixel(wx: float, wy: float):
    """Top-down camera projection: robot-frame (x,y) → image (u,v)."""
    u = int(CX + FX * (wy - CAM_Y) / CAM_H)
    v = int(CY - FY * (wx - CAM_X) / CAM_H)
    return u, v


def meters_to_pixels(m: float) -> int:
    return max(int(FX * m / CAM_H), 4)


class CameraSimulator(Node):
    def __init__(self):
        super().__init__('camera_simulator')
        self.declare_parameter('publish_rate', 10.0)
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.img_pub  = self.create_publisher(Image,      '/camera/image_raw',   10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # Prepare ArUco dictionary
        self.aruco_dict   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        self._frame = 0
        self.timer = self.create_timer(1.0 / rate, self.publish_frame)
        self.get_logger().info(
            f'Camera simulator started — {IMG_W}×{IMG_H} @ {rate:.0f} Hz')

    # ------------------------------------------------------------------
    def _build_image(self) -> np.ndarray:
        # Dark grey table background
        img = np.full((IMG_H, IMG_W, 3), 50, dtype=np.uint8)

        # Subtle grid pattern (looks like a marked table)
        for gx in range(0, IMG_W, 40):
            cv2.line(img, (gx, 0), (gx, IMG_H), (60, 60, 60), 1)
        for gy in range(0, IMG_H, 40):
            cv2.line(img, (0, gy), (IMG_W, gy), (60, 60, 60), 1)

        t = self._frame * 0.05   # slow oscillation phase

        for obj in SCENE_OBJECTS:
            # Slight position jitter to simulate imperfect placement
            wx = obj['wx'] + 0.005 * math.sin(t + obj['id'])
            wy = obj['wy'] + 0.004 * math.cos(t * 1.3 + obj['id'])
            cx, cy = world_to_pixel(wx, wy)
            r_px = meters_to_pixels(obj['size'])
            bgr   = COLORS_BGR[obj['color']]

            # Draw shape
            if obj['shape'] == 'rect':
                half = r_px
                pts = np.array([
                    [cx - half, cy - half],
                    [cx + half, cy - half],
                    [cx + half, cy + half],
                    [cx - half, cy + half],
                ], dtype=np.int32)
                cv2.fillPoly(img, [pts], bgr)
                cv2.polylines(img, [pts], True, (255, 255, 255), 1)
            else:  # circle
                cv2.circle(img, (cx, cy), r_px, bgr, -1)
                cv2.circle(img, (cx, cy), r_px, (255, 255, 255), 1)

            # Overlay ArUco marker (30×30 px) at object centre
            marker_px = 30
            marker_img = np.zeros((marker_px, marker_px), dtype=np.uint8)
            cv2.aruco.drawMarker(self.aruco_dict, obj['id'], marker_px, marker_img, 1)
            marker_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)

            mx0 = cx - marker_px // 2
            my0 = cy - marker_px // 2
            mx1 = mx0 + marker_px
            my1 = my0 + marker_px

            # Clip to image bounds before pasting
            if 0 <= mx0 < IMG_W and 0 <= my0 < IMG_H:
                mx1c = min(mx1, IMG_W)
                my1c = min(my1, IMG_H)
                img[my0:my1c, mx0:mx1c] = marker_bgr[:my1c-my0, :mx1c-mx0]

            # Label (colour + shape)
            cv2.putText(img, f"{obj['color'][0].upper()} #{obj['id']}",
                        (cx - 20, cy + r_px + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)

        self._frame += 1
        return img

    def publish_frame(self):
        now = self.get_clock().now().to_msg()
        img = self._build_image()

        # Publish image
        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        img_msg.header.stamp = now
        img_msg.header.frame_id = 'camera_link'
        self.img_pub.publish(img_msg)

        # Publish camera info (needed by vision_detector)
        info = CameraInfo()
        info.header.stamp = now
        info.header.frame_id = 'camera_link'
        info.width  = IMG_W
        info.height = IMG_H
        info.k = [FX, 0.0, CX,
                  0.0, FY, CY,
                  0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.distortion_model = 'plumb_bob'
        self.info_pub.publish(info)


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
