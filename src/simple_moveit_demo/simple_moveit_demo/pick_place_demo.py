#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, 
    Constraints, 
    PositionConstraint, 
    OrientationConstraint,
    WorkspaceParameters
)
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3
from shape_msgs.msg import SolidPrimitive
import sys

class PickPlaceDemo(Node):
    def __init__(self):
        super().__init__('pick_place_demo')
        
        # Create action client for MoveGroup
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        
        self.get_logger().info('Waiting for MoveGroup action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('MoveGroup action server found!')
        
        # Run the demo
        self.run_demo()
    
    def run_demo(self):
        """Run the pick and place demonstration"""
        try:
            self.get_logger().info('Starting pick and place demo...')
            
            # Define poses
            pick_pose = PoseStamped()
            pick_pose.header.frame_id = "panda_link0"
            pick_pose.pose.position = Point(x=0.4, y=0.1, z=0.3)
            pick_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            place_pose = PoseStamped()
            place_pose.header.frame_id = "panda_link0"
            place_pose.pose.position = Point(x=0.4, y=-0.1, z=0.3)
            place_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            # Execute pick and place sequence
            self.move_to_pose(pick_pose, "pick")
            self.move_to_pose(place_pose, "place")
            
            self.get_logger().info('Pick and place demo completed!')
            
        except Exception as e:
            self.get_logger().error(f'Demo failed: {str(e)}')
    
    def move_to_pose(self, target_pose, description):
        """Move the robot arm to a target pose"""
        try:
            self.get_logger().info(f'Moving to {description} position...')
            
            # Create the action goal
            goal_msg = MoveGroup.Goal()
            
            # Set up the motion plan request
            goal_msg.request.group_name = "panda_arm"
            goal_msg.request.num_planning_attempts = 10
            goal_msg.request.allowed_planning_time = 5.0
            goal_msg.request.max_velocity_scaling_factor = 0.1
            goal_msg.request.max_acceleration_scaling_factor = 0.1
            
            # Create constraints
            constraint = Constraints()
            
            # Position constraint
            pos_constraint = PositionConstraint()
            pos_constraint.header = target_pose.header
            pos_constraint.link_name = "panda_hand"
            pos_constraint.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
            
            # Define the constraint region (small box around target)
            constraint_region = SolidPrimitive()
            constraint_region.type = SolidPrimitive.BOX
            constraint_region.dimensions = [0.01, 0.01, 0.01]  # 1cm tolerance
            
            pos_constraint.constraint_region.primitives.append(constraint_region)
            pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)
            pos_constraint.weight = 1.0
            
            constraint.position_constraints.append(pos_constraint)
            
            # Orientation constraint
            orient_constraint = OrientationConstraint()
            orient_constraint.header = target_pose.header
            orient_constraint.link_name = "panda_hand"
            orient_constraint.orientation = target_pose.pose.orientation
            orient_constraint.absolute_x_axis_tolerance = 0.1
            orient_constraint.absolute_y_axis_tolerance = 0.1
            orient_constraint.absolute_z_axis_tolerance = 0.1
            orient_constraint.weight = 1.0
            
            constraint.orientation_constraints.append(orient_constraint)
            
            # Add constraint to goal
            goal_msg.request.goal_constraints.append(constraint)
            
            # Set workspace parameters - FIXED: Use Vector3 instead of Point
            workspace = WorkspaceParameters()
            workspace.header.frame_id = "panda_link0"
            workspace.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
            workspace.max_corner = Vector3(x=1.0, y=1.0, z=1.0)
            goal_msg.request.workspace_parameters = workspace
            
            # Send the goal
            self.get_logger().info(f'Sending {description} goal...')
            future = self._action_client.send_goal_async(goal_msg)
            
            # Wait for the action to complete
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error(f'{description} goal was rejected')
                return False
            
            self.get_logger().info(f'{description} goal accepted, waiting for result...')
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            result = result_future.result()
            if result.result.error_code.val == 1:  # SUCCESS
                self.get_logger().info(f'{description} motion completed successfully!')
                return True
            else:
                self.get_logger().error(f'{description} motion failed with error code: {result.result.error_code.val}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error in move_to_pose: {str(e)}')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    demo = None
    try:
        demo = PickPlaceDemo()
        rclpy.spin(demo)
    except KeyboardInterrupt:
        print("Demo interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if demo is not None:
            demo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()