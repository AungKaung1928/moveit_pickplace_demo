#!/usr/bin/env python3

import time
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene, ObjectColor
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

BALL_Z = 0.025
BALL_RADIUS = 0.03

BOX_X, BOX_Y = 0.55, 0.25
BOX_HALF  = 0.07
BOX_WALL_H = 0.07
BOX_FLOOR_Z = 0.005

# 2×2 grid of slot offsets inside the box, then stacked layers above
_BOX_SLOTS = [(-0.025, -0.025), (0.025, -0.025), (-0.025, 0.025), (0.025, 0.025)]

# 10 pre-defined reachable positions on the table (panda_link0 frame, metres).
# All within [0.18, 0.82] reach. All clear of box region (x∈[0.48,0.62], y∈[0.18,0.32]).
_BALL_POSITIONS = [
    (0.40, -0.30), (0.52, -0.30), (0.64, -0.25),
    (0.40, -0.17), (0.52, -0.17), (0.64, -0.13),
    (0.40, -0.04), (0.52, -0.04),
    (0.40,  0.09), (0.52,  0.09),
]


def _pose(x: float, y: float, z: float) -> Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.w = 1.0
    return p


class SceneManager:
    """N red balls + one orange box. Balls disappear from scene when placed."""

    def __init__(self, node: Node):
        self._node = node
        self._col_pub    = node.create_publisher(CollisionObject, '/collision_object', 10)
        self._scene_pub  = node.create_publisher(PlanningScene,   '/planning_scene',   10)
        self._marker_pub = node.create_publisher(MarkerArray,     '/ball_markers',     10)
        self._ball_queue: list[tuple[str, float, float, float]] = []
        self._placed_count: int = 0

    def setup_scene(self, ball_count: int = 7) -> None:
        ball_count = max(1, min(len(_BALL_POSITIONS), ball_count))
        self._placed_count = 0
        time.sleep(1.0)
        self._add_box('table', 0.5, 0.0, -0.025, 1.0, 1.0, 0.05)
        self._ball_queue = []
        for i in range(ball_count):
            bx, by = _BALL_POSITIONS[i]
            ball_id = f'ball_{i}'
            self._add_sphere(ball_id, bx, by, BALL_Z, BALL_RADIUS)
            self._ball_queue.append((ball_id, bx, by, BALL_Z))
        self._add_container()
        time.sleep(0.5)
        self._set_colors(ball_count)
        self._node.get_logger().info(f'Scene ready — {ball_count} balls, 1 box.')

    def next_ball(self) -> tuple[str, float, float, float] | None:
        if not self._ball_queue:
            return None
        return self._ball_queue.pop(0)

    def remaining(self) -> int:
        return len(self._ball_queue)

    def remove_object(self, obj_id: str) -> None:
        obj = CollisionObject()
        obj.header.frame_id = 'panda_link0'
        obj.header.stamp = self._node.get_clock().now().to_msg()
        obj.id = obj_id
        obj.operation = CollisionObject.REMOVE
        self._col_pub.publish(obj)

    def show_ghost_ball(self, obj_id: str, x: float, y: float, z: float) -> None:
        m = Marker()
        m.header.frame_id = 'panda_link0'
        m.header.stamp = self._node.get_clock().now().to_msg()
        m.ns = 'ghost_balls'
        m.id = int(obj_id.split('_')[1])
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.x = BALL_RADIUS * 2
        m.scale.y = BALL_RADIUS * 2
        m.scale.z = BALL_RADIUS * 2
        m.color.r = 1.0
        m.color.g = 0.15
        m.color.b = 0.10
        m.color.a = 1.0
        arr = MarkerArray()
        arr.markers.append(m)
        self._marker_pub.publish(arr)

    def hide_ghost_ball(self, obj_id: str) -> None:
        m = Marker()
        m.header.frame_id = 'panda_link0'
        m.header.stamp = self._node.get_clock().now().to_msg()
        m.ns = 'ghost_balls'
        m.id = int(obj_id.split('_')[1])
        m.action = Marker.DELETE
        arr = MarkerArray()
        arr.markers.append(m)
        self._marker_pub.publish(arr)

    def place_in_box(self, obj_id: str) -> None:
        slot = self._placed_count
        layer = slot // 4
        ox, oy = _BOX_SLOTS[slot % 4]
        bz = BOX_FLOOR_Z + BALL_RADIUS + layer * (BALL_RADIUS * 2)
        self._add_sphere(obj_id, BOX_X + ox, BOX_Y + oy, bz, BALL_RADIUS)
        self._set_color(obj_id, 1.0, 0.15, 0.10, 1.0)
        self._placed_count += 1
        self._node.get_logger().info(
            f'[SCENE] {obj_id} placed in box — slot {slot} at z={bz:.3f}m.')

    # ── private helpers ───────────────────────────────────────────────────────

    def _add_container(self) -> None:
        h, half, t = BOX_WALL_H, BOX_HALF, 0.008
        bx, by = BOX_X, BOX_Y
        z = h / 2.0
        self._add_box('box_floor', bx,        by,        BOX_FLOOR_Z / 2, 0.14, 0.14, BOX_FLOOR_Z)
        self._add_box('box_n',     bx + half,  by,        z,               t,    0.14, h)
        self._add_box('box_s',     bx - half,  by,        z,               t,    0.14, h)
        self._add_box('box_e',     bx,         by + half, z,               0.14, t,    h)
        self._add_box('box_w',     bx,         by - half, z,               0.14, t,    h)

    def _add_sphere(self, obj_id: str, x: float, y: float, z: float, radius: float) -> None:
        obj = CollisionObject()
        obj.header.frame_id = 'panda_link0'
        obj.header.stamp = self._node.get_clock().now().to_msg()
        obj.id = obj_id
        obj.operation = CollisionObject.ADD
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [radius]
        obj.primitives.append(prim)
        obj.primitive_poses.append(_pose(x, y, z))
        self._col_pub.publish(obj)

    def _add_box(self, obj_id: str, x: float, y: float, z: float,
                 lx: float, ly: float, lz: float) -> None:
        obj = CollisionObject()
        obj.header.frame_id = 'panda_link0'
        obj.header.stamp = self._node.get_clock().now().to_msg()
        obj.id = obj_id
        obj.operation = CollisionObject.ADD
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [lx, ly, lz]
        obj.primitives.append(prim)
        obj.primitive_poses.append(_pose(x, y, z))
        self._col_pub.publish(obj)

    def _set_color(self, obj_id: str, r: float, g: float, b: float, a: float = 1.0) -> None:
        ps = PlanningScene()
        ps.is_diff = True
        c = ObjectColor()
        c.id = obj_id
        c.color = ColorRGBA(r=r, g=g, b=b, a=a)
        ps.object_colors.append(c)
        self._scene_pub.publish(ps)

    def _set_colors(self, ball_count: int = 1) -> None:
        self._set_color('table', 0.55, 0.35, 0.10, 1.0)
        for i in range(ball_count):
            self._set_color(f'ball_{i}', 1.0, 0.15, 0.10, 1.0)
        for part in ('box_floor', 'box_n', 'box_s', 'box_e', 'box_w'):
            self._set_color(part, 1.0, 0.60, 0.0, 1.0)
