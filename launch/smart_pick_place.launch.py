"""
Smart pick-and-place demo launch file.

Starts:
  1. MoveIt2 move_group + robot_state_publisher + static TF
  2. RViz2
  3. camera_simulator     — synthetic top-down camera
  4. vision_detector      — OpenCV + ArUco + PyTorch classifier
  5. workspace_validator  — C++ reachability filter
  6. smart_pick_place     — FSM controller with colour sorting
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = get_package_share_directory('simple_moveit_demo')

    moveit_config = (
        MoveItConfigsBuilder("panda",
                             package_name="moveit_resources_panda_moveit_config")
        .to_moveit_configs()
    )

    # ── args ────────────────────────────────────────────────────────────────
    use_sim_camera = DeclareLaunchArgument(
        'use_sim_camera', default_value='true',
        description='Launch synthetic camera simulator (false = real camera)')

    # ── MoveIt2 core ────────────────────────────────────────────────────────
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description],
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'panda_link0'],
        output='log',
    )

    # Camera → panda_link0 static TF (camera mounted ~80 cm above, looking down)
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        # x=0.40, y=0, z=0.80 relative to panda_link0, pointing straight down
        # RPY: roll=pi (flip Z) so camera looks downward
        arguments=['0.40', '0', '0.80', '3.14159', '0', '0',
                   'panda_link0', 'camera_link'],
        output='log',
    )

    # ── RViz ────────────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_share, 'rviz', 'vision_moveit.rviz')
    rviz_args   = ['-d', rviz_config] if os.path.exists(rviz_config) else []

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=rviz_args,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # ── Vision pipeline ─────────────────────────────────────────────────────
    camera_sim = Node(
        package='simple_moveit_demo',
        executable='camera_simulator',
        name='camera_simulator',
        output='screen',
        parameters=[{'publish_rate': 10.0}],
        condition=None,   # always start in this demo
    )

    vision_detector = Node(
        package='simple_moveit_demo',
        executable='vision_detector',
        name='vision_detector',
        output='screen',
    )

    # ── C++ workspace validator ─────────────────────────────────────────────
    workspace_validator = Node(
        package='moveit_grasp_utils',
        executable='workspace_validator',
        name='workspace_validator',
        output='screen',
        parameters=[{
            'max_reach':   0.82,
            'min_reach':   0.18,
            'max_height':  0.98,
            'min_height': -0.05,
        }],
    )

    # ── Smart pick-place controller (delayed 3 s to let MoveIt warm up) ─────
    smart_controller = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='simple_moveit_demo',
                executable='smart_pick_place',
                name='smart_pick_place',
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        use_sim_camera,
        move_group,
        robot_state_pub,
        static_tf,
        camera_tf,
        rviz,
        camera_sim,
        vision_detector,
        workspace_validator,
        smart_controller,
    ])
