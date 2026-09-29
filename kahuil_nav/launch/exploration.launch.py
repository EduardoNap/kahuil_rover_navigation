import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_dir = get_package_share_directory('kahuil_nav')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_zed_odom = LaunchConfiguration('use_zed_odom')

    # Include SLAM + Nav2
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'slam.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_zed_odom': use_zed_odom,
        }.items()
    )

    # Custom frontier explorer (ported from rastec)
    frontier_explorer = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_dir, 'scripts', 'frontier_explorer.py')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_zed_odom',
            default_value='false',
            description='Use ZED VIO + EKF instead of RTAB-Map visual odom'),

        slam_launch,
        frontier_explorer,
    ])
