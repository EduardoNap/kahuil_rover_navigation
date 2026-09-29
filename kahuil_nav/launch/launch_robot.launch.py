import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_dir = get_package_share_directory('kahuil_nav')

    # ============ Robot State Publisher (URDF + static TFs) ============
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'use_ros2_control': 'true',
            'robot_model': 'rover',
        }.items()
    )

    # ============ Robot description (passed to ros2_control_node) ============
    # We re-build the URDF here because controller_manager needs robot_description
    # as a parameter (not from /robot_description topic).
    rover_xacro = os.path.join(pkg_dir, 'description', 'rover.urdf.xacro')
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', rover_xacro,
                     ' use_ros2_control:=true',
                     ' sim_mode:=false']),
            value_type=str)
    }

    # ============ Controller Manager (ros2_control) ============
    controller_params_file = os.path.join(pkg_dir, 'config', 'my_controllers.yaml')

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_params_file],
        # Map diff_drive_controller input to /cmd_vel so twist_mux publishes
        # there directly (use_stamped_vel: false in YAML).
        # /diff_cont/odom is exposed as /odom for diagnostics only — ZED VIO
        # owns the odom→base_footprint TF (enable_odom_tf:false in YAML).
        remappings=[
            ('/diff_cont/cmd_vel_unstamped', '/cmd_vel'),
            ('/diff_cont/odom', '/odom'),
        ],
        output='screen',
    )

    # Give RSP / xacro a moment before controller_manager spins up.
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # ============ Controller spawners ============
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
    )

    # Wait until controller_manager is up before spawning controllers.
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # ============ Twist Mux ============
    twist_mux_params = os.path.join(pkg_dir, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel_out', '/cmd_vel')],
        output='screen',
    )

    # ============ RPLidar A1 (sllidar_ros2 — Slamtec official driver) ============
    # NOTE: sllidar_ros2 lives in ~/ros2_ws — that workspace MUST be sourced
    # before launching this file:
    #   source ~/ros2_ws/install/setup.bash
    #   source ~/Desktop/kahuil_new_rover/kahuil_ws/install/setup.bash
    # CH341-based USB-serial gives a stable /dev/ttyCH341USB0 (separate kernel
    # driver from ttyUSB*), so it never races with the ESP32 on /dev/ttyUSB0.
    sllidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch', 'sllidar_a1_launch.py',
            )
        ]),
        launch_arguments={
            'serial_port': '/dev/ttyCH341USB0',
            'frame_id':    'lidar_link',
        }.items()
    )

    return LaunchDescription([
        rsp,
        twist_mux,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        sllidar,
    ])
