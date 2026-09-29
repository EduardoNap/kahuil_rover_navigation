import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    robot_model = LaunchConfiguration('robot_model')

    pkg_path = os.path.join(get_package_share_directory('kahuil_nav'))

    # Select URDF based on robot_model: 'wheeled' or 'rover' (legacy alias 'tracked' also loads rover)
    robot_description_config = Command([
        'bash -c "',
        'case \\"$(echo ', robot_model, ')\\" in ',
        'rover|tracked) ',
        'xacro ', os.path.join(pkg_path, 'description', 'rover.urdf.xacro'),
        ' use_ros2_control:=', use_ros2_control,
        ' sim_mode:=', use_sim_time, ' ;; ',
        '*) ',
        'xacro ', os.path.join(pkg_path, 'description', 'robot.urdf.xacro'),
        ' use_ros2_control:=', use_ros2_control,
        ' sim_mode:=', use_sim_time, ' ;; ',
        'esac"',
    ])

    params = {
        'robot_description': ParameterValue(robot_description_config, value_type=str),
        'use_sim_time': use_sim_time,
    }
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='false',
            description='Use ros2_control if true'),

        DeclareLaunchArgument(
            'robot_model',
            default_value='wheeled',
            description='Robot model to use: wheeled or rover'),

        node_robot_state_publisher
    ])
