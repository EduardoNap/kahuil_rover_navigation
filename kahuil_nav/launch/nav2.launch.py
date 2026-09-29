"""
Nav2 navigation stack only (no AMCL).

Use this WITH slam_toolbox running, since slam_toolbox already publishes
the map -> odom TF. AMCL is therefore not needed.

Bring-up sequence:
    Terminal A:  ros2 launch kahuil_nav launch_robot.launch.py
    Terminal B:  ros2 launch kahuil_nav online_async_slam.launch.py
    Terminal C:  ros2 launch kahuil_nav nav2.launch.py

In RViz: Fixed Frame = map; add Map (/map), LaserScan (/scan), Costmaps,
TF, RobotModel; click 2D Nav Goal to send a target.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('kahuil_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart    = LaunchConfiguration('autostart')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart',    default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file':  nav2_params,
                'autostart':    autostart,
            }.items()
        ),
    ])
