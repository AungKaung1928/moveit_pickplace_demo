#!/usr/bin/env python3
"""Pick-and-place FSM: GRASPING → PLACING → (next ball) or DONE. HOMING on failure only."""

import threading
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

from simple_moveit_demo.grasp_planner import GraspPlanner
from simple_moveit_demo.scene_manager import SceneManager


class State(Enum):
    IDLE     = auto()
    GRASPING = auto()
    PLACING  = auto()
    HOMING   = auto()
    DONE     = auto()
    ERROR    = auto()


PLAN_ATTEMPTS = 20
PLAN_TIME     = 15.0
VEL_SCALE     = 0.3
ACCEL_SCALE   = 0.3


class SmartPickPlace(Node):

    def __init__(self):
        super().__init__('smart_pick_place')

        self.declare_parameter('ball_count', 7)
        self._ball_count = int(self.get_parameter('ball_count').value)

        self._planner    = GraspPlanner()
        self._scene      = SceneManager(self)
        self._state      = State.IDLE
        self._state_lock = threading.Lock()
        self._server_ready = False

        self._pending_target: PoseStamped | None = None
        self._current_ball_id: str | None = None

        cb_group = ReentrantCallbackGroup()
        self._ac = ActionClient(self, MoveGroup, '/move_action', callback_group=cb_group)

        self._status_pub = self.create_publisher(String, '/pick_place_status', 10)

        self.create_timer(1.0, self._check_server_cb)
        self.get_logger().info(
            f'SmartPickPlace started — {self._ball_count} balls — waiting for MoveGroup...')

    def _check_server_cb(self):
        if self._server_ready:
            return
        if self._ac.server_is_ready():
            self._server_ready = True
            self.get_logger().info(
                f'MoveGroup connected — spawning {self._ball_count} balls.')
            threading.Thread(target=self._setup_and_start, daemon=True).start()
        else:
            self.get_logger().info('Waiting for MoveGroup action server...')

    def _set_state(self, s: State):
        with self._state_lock:
            self._state = s
        msg = String()
        msg.data = s.name
        self._status_pub.publish(msg)
        self.get_logger().info(f'[STATE] → {s.name}')

    def _get_state(self) -> State:
        with self._state_lock:
            return self._state

    def _setup_and_start(self) -> None:
        self._scene.setup_scene(self._ball_count)
        self._start_next_cycle()

    def _start_next_cycle(self) -> None:
        ball_info = self._scene.next_ball()
        if ball_info is None:
            self._set_state(State.DONE)
            self.get_logger().info(
                f'All {self._ball_count} balls placed in box. Process complete.')
            return
        ball_id, bx, by, bz = ball_info
        remaining = self._scene.remaining()
        self.get_logger().info(
            f'Picking {ball_id} at ({bx:.3f}, {by:.3f}) — '
            f'{remaining} remaining after this.')
        target = PoseStamped()
        target.header.frame_id = 'panda_link0'
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x = bx
        target.pose.position.y = by
        target.pose.position.z = bz
        target.pose.orientation = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)
        self._pending_target = target
        self._current_ball_id = ball_id
        threading.Thread(target=self._execute_sequence, daemon=True).start()

    def _execute_sequence(self):
        try:
            ball_id = self._current_ball_id
            target  = self._pending_target
            ox, oy, oz = (target.pose.position.x,
                          target.pose.position.y,
                          target.pose.position.z)

            _, grasp_pose, _ = self._planner.grasp_sequence(ox, oy, oz)

            self._scene.show_ghost_ball(ball_id, ox, oy, oz)
            self._scene.remove_object(ball_id)
            self._set_state(State.GRASPING)
            grasped = self._move(grasp_pose, pos_tol=0.04)
            self._scene.hide_ghost_ball(ball_id)
            if not grasped:
                self._recover(); return

            self._set_state(State.PLACING)
            if not self._move(self._planner.box_pose(), pos_tol=0.06):
                self._recover(); return
            self._scene.place_in_box(ball_id)

            self.get_logger().info(f'{ball_id} placed in box.')
            self._start_next_cycle()

        except Exception as e:
            self.get_logger().error(f'Execution error: {e}')
            self._recover()

    def _recover(self):
        self.get_logger().warn('Recovery — moving home.')
        self._set_state(State.HOMING)
        if self._current_ball_id:
            self._scene.hide_ghost_ball(self._current_ball_id)
        self._move(self._planner.home_pose(), pos_tol=0.06)
        self._start_next_cycle()

    def _move(self, target: PoseStamped, pos_tol: float = 0.01) -> bool:
        try:
            goal = MoveGroup.Goal()
            goal.planning_options.plan_only       = False
            goal.planning_options.replan          = True
            goal.planning_options.replan_attempts = 3
            req = goal.request
            req.group_name                      = 'panda_arm'
            req.num_planning_attempts           = PLAN_ATTEMPTS
            req.allowed_planning_time           = PLAN_TIME
            req.max_velocity_scaling_factor     = VEL_SCALE
            req.max_acceleration_scaling_factor = ACCEL_SCALE

            pc = PositionConstraint()
            pc.header    = target.header
            pc.link_name = 'panda_link8'
            pc.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [pos_tol, pos_tol, pos_tol]
            pc.constraint_region.primitives.append(box)
            pc.constraint_region.primitive_poses.append(target.pose)
            pc.weight = 1.0

            c = Constraints()
            c.position_constraints.append(pc)
            req.goal_constraints.append(c)

            done_event    = threading.Event()
            result_holder = [None]

            def goal_response_cb(future):
                handle = future.result()
                if not handle.accepted:
                    self.get_logger().error('Goal rejected by MoveGroup')
                    done_event.set()
                    return
                handle.get_result_async().add_done_callback(
                    lambda f: (result_holder.__setitem__(0, f.result()), done_event.set()))

            self._ac.send_goal_async(goal).add_done_callback(goal_response_cb)
            done_event.wait(timeout=60.0)

            if result_holder[0] is None:
                self.get_logger().error('MoveGroup goal timed out.')
                return False

            ec = result_holder[0].result.error_code.val
            ok = ec == 1
            if not ok:
                self.get_logger().warn(f'MoveGroup error code: {ec}')
            return ok

        except Exception as e:
            self.get_logger().error(f'_move exception: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = SmartPickPlace()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
