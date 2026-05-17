"""
Smart pick-and-place demo launch file.

Starts:
  1. MoveIt2 move_group + robot_state_publisher + static TF
  2. ros2_control (mock hardware) + joint_state_broadcaster + panda_arm_controller
  3. RViz2
  4. camera_simulator     — synthetic top-down camera
  5. vision_detector      — OpenCV + ArUco + PyTorch classifier
  6. workspace_validator  — C++ reachability filter
  7. smart_pick_place     — FSM controller with colour sorting
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
        MoveItConfigsBuilder("moveit_resources_panda",
                             package_name="moveit_resources_panda_moveit_config")
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={"ros2_control_hardware_type": "mock_components"},
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .robot_description_kinematics()
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits()
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
            moveit_config.trajectory_execution,
            moveit_config.moveit_cpp,
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

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        arguments=['0.40', '0', '0.80', '3.14159', '0', '0',
                   'panda_link0', 'camera_link'],
        output='log',
    )

    # ── ros2_control (mock hardware) ─────────────────────────────────────────
    ros2_controllers_path = os.path.join(
        get_package_share_directory('moveit_resources_panda_moveit_config'),
        'config', 'ros2_controllers.yaml',
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ros2_controllers_path],
        remappings=[('/controller_manager/robot_description', '/robot_description')],
        output='screen',
    )

    # Spawners delayed so controller_manager has time to start
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        )],
    )

    panda_arm_controller_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['panda_arm_controller', '-c', '/controller_manager'],
        )],
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

    demo_params = os.path.join(pkg_share, 'config', 'demo_params.yaml')

    # ── Smart pick-place controller (delayed to let MoveIt + controllers warm up)
    smart_controller = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='simple_moveit_demo',
                executable='smart_pick_place',
                name='smart_pick_place',
                output='screen',
                parameters=[demo_params],
            )
        ]
    )

    return LaunchDescription([
        use_sim_camera,
        # MoveIt core
        move_group,
        robot_state_pub,
        static_tf,
        camera_tf,
        # ros2_control mock hardware
        ros2_control_node,
        joint_state_broadcaster_spawner,
        panda_arm_controller_spawner,
        # UI
        rviz,
        # Vision
        camera_sim,
        vision_detector,
        workspace_validator,
        # Controller
        smart_controller,
    ])
