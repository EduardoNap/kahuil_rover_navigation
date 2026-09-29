import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'kahuil_nav'
    pkg_dir = get_package_share_directory(package_name)

    robot_model = LaunchConfiguration('robot_model')

    # Robot state publisher (use_ros2_control=false → Gazebo DiffDrive plugin)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'false',
            'robot_model': robot_model,
        }.items()
    )

    # World file
    default_world = os.path.join(pkg_dir, 'worlds', 'empty.sdf')
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World file to load')

    # Gazebo Ignition
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 ', world],
            'on_exit_shutdown': 'true',
        }.items()
    )

    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'kahuil_robot',
            '-z', '0.1',
        ],
        output='screen')

    # Gazebo-ROS bridge (sensors, odom, TF, cmd_vel)
    bridge_params = os.path.join(pkg_dir, 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p', f'config_file:={bridge_params}',
        ])

    # Image bridges (ros_gz_image handles encoding better than generic bridge)
    ros_gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image'])

    ros_gz_depth_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/depth_image'],
        remappings=[('/camera/depth_image', '/camera/depth/image_raw')])

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='wheeled',
        description='Robot model to use: wheeled or tracked')

    return LaunchDescription([
        robot_model_arg,
        world_arg,
        rsp,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        ros_gz_image_bridge,
        ros_gz_depth_bridge,
    ])
