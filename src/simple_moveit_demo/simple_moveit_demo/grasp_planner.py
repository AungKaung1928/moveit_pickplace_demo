#!/usr/bin/env python3

import math
from geometry_msgs.msg import PoseStamped, Quaternion

_BOX_DROP = (0.55, 0.25, 0.30)

APPROACH_CLEARANCE = 0.20
GRASP_CLEARANCE    = 0.04
LIFT_HEIGHT        = 0.28
_MIN_GRASP_Z       = 0.16   # panda fingertips ~0.12m below panda_link8; keep above table


def _euler_z_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Quaternion:
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

    def grasp_sequence(self, obj_x: float, obj_y: float, obj_z: float = 0.0,
                       grasp_angle_deg: float = 0.0,
                       frame_id: str = 'panda_link0'):
        yaw = grasp_angle_deg
        grasp_z = max(obj_z + GRASP_CLEARANCE, _MIN_GRASP_Z)
        pre_grasp = _make_pose(obj_x, obj_y, obj_z + APPROACH_CLEARANCE, yaw=yaw, frame_id=frame_id)
        grasp     = _make_pose(obj_x, obj_y, grasp_z,                    yaw=yaw, frame_id=frame_id)
        lift      = _make_pose(obj_x, obj_y, obj_z + LIFT_HEIGHT,        yaw=yaw, frame_id=frame_id)
        return [pre_grasp, grasp, lift]

    def box_pose(self, frame_id: str = 'panda_link0') -> PoseStamped:
        x, y, z = _BOX_DROP
        return _make_pose(x, y, z, frame_id=frame_id)

    def home_pose(self, frame_id: str = 'panda_link0') -> PoseStamped:
        return _make_pose(0.3, 0.0, 0.6, frame_id=frame_id)
