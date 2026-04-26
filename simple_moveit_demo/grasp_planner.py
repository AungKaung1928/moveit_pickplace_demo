#!/usr/bin/env python3

import math
from geometry_msgs.msg import PoseStamped, Quaternion

# Bin drop-off positions (frame: panda_link0)
_BINS = {
    'red':       (0.55, -0.28, 0.30),
    'green':     (0.55,  0.28, 0.30),
    'blue':      (0.30, -0.35, 0.30),
    'yellow':    (0.30,  0.35, 0.30),
    'cube':      (0.55, -0.28, 0.30),
    'cylinder':  (0.55,  0.28, 0.30),
    'irregular': (0.30, -0.35, 0.30),
    'default':   (0.40, -0.25, 0.30),
}

APPROACH_CLEARANCE = 0.18   # m above object for pre-grasp
GRASP_CLEARANCE    = 0.04   # m above table during grasp
LIFT_HEIGHT        = 0.28   # m above grasp point


def _euler_z_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Quaternion:
    """Convert RPY (degrees) to quaternion (x, y, z, w)."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)

    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def _make_pose(x: float, y: float, z: float,
               roll=180.0, pitch=0.0, yaw=0.0,
               frame_id: str = 'panda_link0') -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation = _euler_z_to_quaternion(roll, pitch, yaw)
    return pose


class GraspPlanner:
    """
    Compute pre-grasp → grasp → lift → bin pose sequence from a detection.

    grasp_angle_deg aligns the gripper fingers with the object's long axis
    (estimated from the contour's minimum-area bounding rectangle).
    """

    def grasp_sequence(self, obj_x: float, obj_y: float, obj_z: float = 0.0,
                       grasp_angle_deg: float = 0.0,
                       frame_id: str = 'panda_link0'):
        """
        Returns [pre_grasp_pose, grasp_pose, lift_pose].
        All poses approach from above (roll=180° → gripper pointing down).
        """
        yaw = grasp_angle_deg
        grasp_z = max(obj_z + GRASP_CLEARANCE, 0.08)

        pre_grasp = _make_pose(obj_x, obj_y, obj_z + APPROACH_CLEARANCE,
                               yaw=yaw, frame_id=frame_id)
        grasp     = _make_pose(obj_x, obj_y, grasp_z,
                               yaw=yaw, frame_id=frame_id)
        lift      = _make_pose(obj_x, obj_y, obj_z + LIFT_HEIGHT,
                               yaw=yaw, frame_id=frame_id)
        return [pre_grasp, grasp, lift]

    def bin_pose(self, label: str,
                 frame_id: str = 'panda_link0') -> PoseStamped:
        """Return drop-off pose for a given colour/shape label."""
        x, y, z = _BINS.get(label, _BINS['default'])
        return _make_pose(x, y, z, frame_id=frame_id)

    def home_pose(self, frame_id: str = 'panda_link0') -> PoseStamped:
        """Safe home position above the workspace."""
        return _make_pose(0.3, 0.0, 0.6, frame_id=frame_id)
