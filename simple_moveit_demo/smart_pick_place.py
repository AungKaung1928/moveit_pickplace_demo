#!/usr/bin/env python3
"""
Smart Pick-and-Place with FSM + colour-based sorting.

Flow:
  IDLE → SCANNING → PRE_GRASPING → GRASPING → LIFTING
       → PLACING → HOMING → SCANNING (next object)

Subscribes to:
  /validated_targets  geometry_msgs/PoseArray  (C++ workspace validator)
  /detected_objects   visualization_msgs/MarkerArray (for colour labels)

Executes motions via MoveIt2 /move_action ActionClient.
Objects are sorted to different bins by detected colour.
"""

import threading
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, PositionConstraint, WorkspaceParameters,
)
from geometry_msgs.msg import (
    PoseStamped, Vector3, Quaternion, PoseArray,
)
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String

from simple_moveit_demo.grasp_planner import GraspPlanner


class State(Enum):
    IDLE          = auto()
    SCANNING      = auto()
    PRE_GRASPING  = auto()
    GRASPING      = auto()
    LIFTING       = auto()
    PLACING       = auto()
    HOMING        = auto()
    ERROR         = auto()


PLAN_ATTEMPTS  = 20
PLAN_TIME      = 15.0
VEL_SCALE      = 0.3
ACCEL_SCALE    = 0.3
WORKSPACE_HALF = 1.2     # metres


class SmartPickPlace(Node):

    def __init__(self):
        super().__init__('smart_pick_place')

        self._planner    = GraspPlanner()
        self._state      = State.IDLE
        self._state_lock = threading.Lock()
        self._server_ready = False

        self._pending_target: PoseStamped | None = None
        self._pending_label : str = 'unknown'
        self._grasp_angle   : float = 0.0

        cb_group = ReentrantCallbackGroup()
        self._ac = ActionClient(self, MoveGroup, '/move_action',
                                callback_group=cb_group)

        self._status_pub = self.create_publisher(String, '/pick_place_status', 10)
        self._debug_pub  = self.create_publisher(String, '/pick_place_debug',  10)
        self.create_subscription(PoseArray,   '/validated_targets',  self._targets_cb, 10)
        self.create_subscription(MarkerArray, '/detected_objects',   self._detections_cb, 10)

        self._detection_labels: list[tuple] = []
        self._detection_lock = threading.Lock()

        # Poll for MoveGroup server after spin starts (can't block in __init__)
        self.create_timer(1.0, self._check_server_cb)
        self.get_logger().info('SmartPickPlace started — waiting for MoveGroup action server...')

    # ── startup poll ──────────────────────────────────────────────────────────
    def _check_server_cb(self):
        if self._server_ready:
            return
        if self._ac.server_is_ready():
            self._server_ready = True
            self.get_logger().info('MoveGroup connected — entering SCANNING state.')
            self._set_state(State.SCANNING)
        else:
            self.get_logger().info('Still waiting for MoveGroup action server...')

    # ── state helpers ─────────────────────────────────────────────────────────
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

    # ── callbacks ─────────────────────────────────────────────────────────────
    def _detections_cb(self, msg: MarkerArray):
        with self._detection_lock:
            self._detection_labels = [
                (m.pose.position.x, m.pose.position.y, m.ns)
                for m in msg.markers
            ]

    def _targets_cb(self, msg: PoseArray):
        if not self._server_ready:
            return
        if self._get_state() != State.SCANNING:
            return
        if not msg.poses:
            return

        pose_in = msg.poses[0]
        label = self._nearest_label(pose_in.position.x, pose_in.position.y)

        target = PoseStamped()
        target.header.frame_id = 'panda_link0'
        target.header.stamp    = self.get_clock().now().to_msg()
        target.pose            = pose_in
        target.pose.orientation = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)

        self._pending_target = target
        self._pending_label  = label
        self._grasp_angle    = 0.0

        # Claim state before spawning thread to prevent duplicate starts
        self._set_state(State.PRE_GRASPING)

        self.get_logger().info(
            f'Target accepted: {label} @ '
            f'({pose_in.position.x:.3f}, {pose_in.position.y:.3f})')

        threading.Thread(target=self._execute_sequence, daemon=True).start()

    def _nearest_label(self, tx: float, ty: float) -> str:
        with self._detection_lock:
            labels = list(self._detection_labels)
        if not labels:
            return 'unknown'
        best = min(labels, key=lambda t: (t[0]-tx)**2 + (t[1]-ty)**2)
        return best[2]

    # ── sequence execution ────────────────────────────────────────────────────
    def _execute_sequence(self):
        try:
            target = self._pending_target
            label  = self._pending_label
            angle  = self._grasp_angle

            obj_x = target.pose.position.x
            obj_y = target.pose.position.y
            obj_z = target.pose.position.z

            seq = self._planner.grasp_sequence(obj_x, obj_y, obj_z, angle)
            pre_grasp, grasp_pose, lift_pose = seq

            colour_key = label.split('_')[0] if '_' in label else label

            # PRE_GRASP (state already set in _targets_cb)
            if not self._move(pre_grasp, pos_tol=0.06):
                self._recover()
                return

            # GRASP
            self._set_state(State.GRASPING)
            if not self._move(grasp_pose, pos_tol=0.04):
                self._recover()
                return
            self._simulate_gripper('close')

            # LIFT
            self._set_state(State.LIFTING)
            if not self._move(lift_pose, pos_tol=0.06):
                self._recover()
                return

            # PLACE
            self._set_state(State.PLACING)
            bin_pose = self._planner.bin_pose(colour_key)
            if not self._move(bin_pose, pos_tol=0.06):
                self._recover()
                return
            self._simulate_gripper('open')

            # HOME
            self._set_state(State.HOMING)
            self._move(self._planner.home_pose(), pos_tol=0.06)

            self.get_logger().info(f'Cycle complete — placed [{label}] in [{colour_key}] bin.')
            self._set_state(State.SCANNING)

        except Exception as e:
            self.get_logger().error(f'Execution error: {e}')
            self._recover()

    def _recover(self):
        self.get_logger().warn('Attempting recovery — moving home.')
        self._set_state(State.HOMING)
        self._move(self._planner.home_pose(), pos_tol=0.06)
        self._set_state(State.SCANNING)

    def _simulate_gripper(self, action: str):
        self.get_logger().info(f'[GRIPPER] {action.upper()} — '
                               '(real gripper action would fire here)')

    # ── MoveIt helper ─────────────────────────────────────────────────────────
    def _move(self, target: PoseStamped, pos_tol: float = 0.01) -> bool:
        """Send one MoveGroup goal from a background thread.

        Uses threading.Event + add_done_callback so the executor remains free
        to process action responses while this thread blocks on the event.
        """
        try:
            goal = MoveGroup.Goal()
            goal.planning_options.plan_only             = False
            goal.planning_options.replan                = True
            goal.planning_options.replan_attempts       = 3
            req  = goal.request
            req.group_name                      = 'panda_arm'
            req.num_planning_attempts           = PLAN_ATTEMPTS
            req.allowed_planning_time           = PLAN_TIME
            req.max_velocity_scaling_factor     = VEL_SCALE
            req.max_acceleration_scaling_factor = ACCEL_SCALE

            c = Constraints()

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
            c.position_constraints.append(pc)

            req.goal_constraints.append(c)

            ws = WorkspaceParameters()
            ws.header.frame_id = 'panda_link0'
            ws.min_corner = Vector3(x=-WORKSPACE_HALF, y=-WORKSPACE_HALF, z=-0.2)
            ws.max_corner = Vector3(x= WORKSPACE_HALF, y= WORKSPACE_HALF, z=1.2)
            req.workspace_parameters = ws

            done_event   = threading.Event()
            result_holder = [None]

            def goal_response_cb(future):
                handle = future.result()
                if not handle.accepted:
                    dbg = String(); dbg.data = 'GOAL_REJECTED by MoveGroup'
                    self._debug_pub.publish(dbg)
                    self.get_logger().error('Goal rejected by MoveGroup')
                    done_event.set()
                    return

                def result_cb(res_future):
                    result_holder[0] = res_future.result()
                    done_event.set()

                handle.get_result_async().add_done_callback(result_cb)

            self._ac.send_goal_async(goal).add_done_callback(goal_response_cb)
            done_event.wait(timeout=60.0)

            if result_holder[0] is None:
                dbg = String(); dbg.data = 'TIMEOUT_OR_REJECTED'
                self._debug_pub.publish(dbg)
                self.get_logger().error('MoveGroup goal timed out or was rejected')
                return False

            ec = result_holder[0].result.error_code.val
            dbg = String(); dbg.data = f'error_code={ec}'
            self._debug_pub.publish(dbg)
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
