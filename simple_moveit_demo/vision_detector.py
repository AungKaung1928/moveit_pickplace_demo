#!/usr/bin/env python3
"""
Vision detector node — combines:
  • HSV multi-colour segmentation  (red / green / blue / yellow)
  • ArUco marker pose estimation   (precise 3-D pose when markers visible)
  • Contour shape-feature extraction
  • PyTorch shape classification   (cube / cylinder / irregular)

Publishes:
  /detected_objects   visualization_msgs/MarkerArray  → C++ workspace validator
  /detection_image    sensor_msgs/Image               → RViz image display
  /target_position    geometry_msgs/Point             → legacy simple pick-place
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np

from simple_moveit_demo.shape_classifier import YOLODetector, geometric_classify

# ── Camera intrinsics (updated dynamically from /camera/camera_info) ──────────
_DEFAULT_K = [520.0, 0.0, 320.0,
              0.0,   520.0, 240.0,
              0.0,   0.0,   1.0]

# Camera mounted looking straight down
CAM_X, CAM_Y, CAM_H = 0.40, 0.00, 0.80   # meters in panda_link0 frame
MARKER_SIZE_M = 0.030                       # ArUco marker physical side length

MIN_CONTOUR_AREA_PX = 400   # filter tiny noise

# HSV ranges [lower, upper] for each colour
HSV_RANGES = {
    'red':    [(np.array([0,   120, 80]),  np.array([10,  255, 255])),
               (np.array([170, 120, 80]),  np.array([180, 255, 255]))],  # red wraps hue
    'green':  [(np.array([40,  80,  60]),  np.array([85,  255, 255]))],
    'blue':   [(np.array([100, 80,  60]),  np.array([135, 255, 255]))],
    'yellow': [(np.array([20,  100, 80]),  np.array([38,  255, 255]))],
}

MARKER_COLORS = {
    'red':    (0.9, 0.1, 0.1),
    'green':  (0.1, 0.9, 0.1),
    'blue':   (0.1, 0.1, 0.9),
    'yellow': (0.9, 0.8, 0.1),
    'aruco':  (0.9, 0.5, 0.0),
}


def _pixel_to_robot(u: float, v: float, fx: float, fy: float,
                    cx: float, cy: float) -> tuple:
    """Project image pixel to robot floor-plane (z=0) using top-down camera model."""
    wy = CAM_Y + (u - cx) * CAM_H / fx
    wx = CAM_X - (v - cy) * CAM_H / fy
    return wx, wy, 0.0


def _circularity(area: float, perimeter: float) -> float:
    if perimeter < 1e-6:
        return 0.0
    return min(4.0 * math.pi * area / (perimeter ** 2), 1.0)


class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_detector')

        # Camera intrinsics (will be overwritten by /camera/camera_info)
        self._K = _DEFAULT_K.copy()
        self._fx = self._K[0]
        self._fy = self._K[4]
        self._cx = self._K[2]
        self._cy = self._K[5]

        self._bridge = CvBridge()
        self._yolo = YOLODetector()   # YOLOv8n pretrained on COCO

        # ArUco setup
        self._aruco_dict   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self._aruco_params = cv2.aruco.DetectorParameters_create()

        # Publishers
        self._marker_pub  = self.create_publisher(MarkerArray, '/detected_objects',  10)
        self._image_pub   = self.create_publisher(Image,       '/detection_image',   10)
        self._target_pub  = self.create_publisher(Point,       '/target_position',   10)

        # Subscribers
        self.create_subscription(CameraInfo, '/camera/camera_info',
                                 self._camera_info_cb, 10)
        self.create_subscription(Image, '/camera/image_raw',
                                 self._image_cb, 10)

        self.get_logger().info('Vision detector initialised — waiting for camera frames...')

    # ── callbacks ─────────────────────────────────────────────────────────────
    def _camera_info_cb(self, msg: CameraInfo):
        K = msg.k
        if len(K) == 9 and K[0] > 0:
            self._fx, self._fy = K[0], K[4]
            self._cx, self._cy = K[2], K[5]

    def _image_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._process(frame, msg.header.stamp)
        except Exception as e:
            self.get_logger().error(f'Image callback error: {e}')

    # ── main processing ───────────────────────────────────────────────────────
    def _process(self, frame: np.ndarray, stamp):
        vis = frame.copy()
        markers_out = MarkerArray()
        detections  = []   # (wx, wy, wz, label)
        marker_id   = 0

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        img_area = frame.shape[0] * frame.shape[1]

        # ── 0. YOLOv8n inference (COCO weights — domain gap expected) ─────────
        yolo_dets = self._yolo.detect(frame)
        for det in yolo_dets:
            x1, y1, x2, y2 = (int(v) for v in det['bbox'])
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(vis, f"YOLO:{det['class']} {det['confidence']:.2f}",
                        (x1, max(y1 - 4, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 255), 1)
        if not yolo_dets:
            cv2.putText(vis, 'YOLO: 0 det (domain gap)',
                        (8, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 180, 255), 1)
            self.get_logger().debug('No YOLO detections — COCO weights vs synthetic top-down camera')
        else:
            self.get_logger().info(f'YOLO: {len(yolo_dets)} object(s) detected')

        # ── 1. ArUco detection ────────────────────────────────────────────────
        corners_list, ids, _ = cv2.aruco.detectMarkers(
            gray, self._aruco_dict, parameters=self._aruco_params)

        if ids is not None:
            cam_mat  = np.array(self._K, dtype=np.float64).reshape(3, 3)
            dist     = np.zeros((5,), dtype=np.float64)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners_list, MARKER_SIZE_M, cam_mat, dist)

            for i, (corners, tvec) in enumerate(zip(corners_list, tvecs)):
                # tvec is in camera frame; convert to robot frame (top-down cam)
                # camera frame: X=right, Y=down, Z=forward (into scene)
                # For top-down: robot_x = cam_Z, robot_y = -cam_X
                cx_img = tvec[0][2]    # depth from camera = height above table
                obj_x  = CAM_X - tvec[0][1]   # cam Y → robot X (inverted)
                obj_y  = CAM_Y + tvec[0][0]    # cam X → robot Y
                obj_z  = 0.0

                label = f'aruco_{ids[i][0]}'
                detections.append((obj_x, obj_y, obj_z, label, 0.0))

                # Draw axis on image
                cv2.aruco.drawAxis(vis, cam_mat, dist,
                                   rvecs[i], tvecs[i], 0.03)
                cv2.putText(vis, f'A{ids[i][0]}',
                            tuple(corners[0][0].astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)

                m = self._make_rviz_marker(
                    marker_id, obj_x, obj_y, obj_z,
                    *MARKER_COLORS['aruco'], stamp, 'aruco')
                markers_out.markers.append(m)
                marker_id += 1

        # ── 2. Colour segmentation ────────────────────────────────────────────
        for color, ranges in HSV_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lo, hi in ranges:
                mask |= cv2.inRange(hsv, lo, hi)

            # Morphological clean-up
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)

            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            for cnt in cnts:
                area_px = cv2.contourArea(cnt)
                if area_px < MIN_CONTOUR_AREA_PX:
                    continue

                # ── shape features ───────────────────────────────────────────
                perim   = cv2.arcLength(cnt, True)
                circ    = _circularity(area_px, perim)
                hull    = cv2.convexHull(cnt)
                solidity= area_px / (cv2.contourArea(hull) + 1e-6)
                rx, ry, rw, rh = cv2.boundingRect(cnt)
                extent  = area_px / (rw * rh + 1e-6)
                aspect  = rw / (rh + 1e-6)
                area_norm = area_px / img_area

                shape_cls, confidence = geometric_classify(circ, aspect, solidity)

                # Minimum-area bounding rect → grasp angle
                (_, _), (_, _), angle = cv2.minAreaRect(cnt)
                grasp_angle = float(angle)   # degrees, −90..0

                # Image centroid
                M  = cv2.moments(cnt)
                if M['m00'] < 1:
                    continue
                u_c = M['m10'] / M['m00']
                v_c = M['m01'] / M['m00']

                # Pixel → robot frame
                wx, wy, wz = _pixel_to_robot(
                    u_c, v_c, self._fx, self._fy, self._cx, self._cy)

                label = f'{color}_{shape_cls}'
                detections.append((wx, wy, wz, label, grasp_angle))

                # Draw annotation
                cv2.drawContours(vis, [cnt], -1, (255, 255, 255), 1)
                cv2.circle(vis, (int(u_c), int(v_c)), 6, (0, 255, 0), -1)
                txt = f'{color} {shape_cls} {confidence:.2f}'
                cv2.putText(vis, txt, (rx, ry - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

                rc = MARKER_COLORS.get(color, (0.7, 0.7, 0.7))
                m  = self._make_rviz_marker(
                    marker_id, wx, wy, wz, *rc, stamp, label)
                markers_out.markers.append(m)
                marker_id += 1

        # ── 3. Publish ────────────────────────────────────────────────────────
        if markers_out.markers:
            self._marker_pub.publish(markers_out)

        if detections:
            # Best target: prefer non-aruco detections, take first
            colour_dets = [(wx, wy, wz, lbl, ang)
                           for wx, wy, wz, lbl, ang in detections
                           if not lbl.startswith('aruco')]
            best = colour_dets[0] if colour_dets else detections[0]
            pt = Point(x=best[0], y=best[1], z=best[2])
            self._target_pub.publish(pt)
            self.get_logger().info(
                f'Detected {len(detections)} objects — best: {best[3]} '
                f'@ ({best[0]:.3f}, {best[1]:.3f}, {best[2]:.3f})')

        # Annotate count on image
        cv2.putText(vis, f'Detections: {len(detections)}',
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        self._image_pub.publish(
            self._bridge.cv2_to_imgmsg(vis, encoding='bgr8'))

    # ── helpers ───────────────────────────────────────────────────────────────
    def _make_rviz_marker(self, mid, x, y, z, r, g, b, stamp, ns='obj'):
        m = Marker()
        m.header.frame_id = 'panda_link0'
        m.header.stamp    = stamp
        m.ns              = ns
        m.id              = mid
        m.type            = Marker.SPHERE
        m.action          = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z + 0.05
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.06
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.85
        m.lifetime.sec = 2
        return m


def main(args=None):
    rclpy.init(args=args)
    node = VisionDetector()
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
