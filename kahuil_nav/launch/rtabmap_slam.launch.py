"""
ZED 2i + RTAB-Map (visual odometry from ZED, depth-only mapping).

Topology:
  ZED publishes /zed/zed_node/odom topic only (publish_tf:false — does NOT
  publish odom->zed_camera_link TF, which would collide with our URDF's
  base_link->zed_camera_link static).
  robot_localization EKF takes /zed/zed_node/odom and uses the URDF static
  chain to publish odom->base_footprint TF.
  RTAB-Map subscribes to rgbd + /odometry/filtered (EKF output) and publishes
  map->odom.

Bring-up sequence:
    Terminal A:  ros2 launch kahuil_nav launch_robot.launch.py
    Terminal B:  ros2 launch kahuil_nav rtabmap_slam.launch.py
                 # add localization:=true to load an existing db instead of mapping
    Terminal C:  ros2 launch kahuil_nav nav2.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    pkg_dir = get_package_share_directory('kahuil_nav')

    rtabmap_params_file = os.path.join(pkg_dir, 'config', 'rtabmap_params.yaml')
    ekf_params_file = os.path.join(pkg_dir, 'config', 'ekf_params.yaml')

    localization = LaunchConfiguration('localization').perform(context).lower() == 'true'
    db_path_str = LaunchConfiguration('db_path').perform(context)
    os.makedirs(os.path.dirname(db_path_str), exist_ok=True)

    # ============ ZED 2i Camera ============
    # publish_tf:false  -> ZED does NOT broadcast odom->zed_camera_link TF.
    #                     The /zed/zed_node/odom topic still publishes (used by EKF).
    # publish_map_tf:false -> RTAB-Map owns the map TF.
    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch', 'zed_camera.launch.py')
        ]),
        launch_arguments={
            'camera_model': 'zed2i',
            'publish_tf': 'false',
            'publish_map_tf': 'false',
        }.items()
    )

    # ============ robot_localization EKF ============
    # Takes /zed/zed_node/odom (camera-frame pose) and publishes odom->base_footprint
    # by transforming through the URDF static chain.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file])

    # ============ RTAB-Map RGBD Sync ============
    rgbd_remappings = [
        ('rgb/image', '/zed/zed_node/rgb/color/rect/image'),
        ('rgb/camera_info', '/zed/zed_node/rgb/color/rect/camera_info'),
        ('depth/image', '/zed/zed_node/depth/depth_registered'),
    ]

    rgbd_sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{
            'approx_sync': True,
            'approx_sync_max_interval': 0.05,
            'queue_size': 30,
            'use_sim_time': False,
        }],
        remappings=rgbd_remappings)

    # ============ RTAB-Map SLAM / Localization ============
    rtabmap_extra_params = {
        'subscribe_rgbd': True,
        'approx_sync': True,
        'wait_for_transform': 1.0,
        'topic_queue_size': 30,
        'sync_queue_size': 30,
        'odom_sensor_sync': True,
        'database_path': db_path_str,
        'use_sim_time': False,
    }

    rtabmap_node_params = [rtabmap_params_file, rtabmap_extra_params]
    if localization:
        rtabmap_node_params.append({
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',
        })
        rtabmap_args = []
    else:
        rtabmap_args = ['-d']  # delete previous db (mapping mode)

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=rtabmap_node_params,
        remappings=[
            ('rgbd_image', '/rgbd_image'),
            ('odom', '/odometry/filtered'),   # from EKF
            # /scan remap kept for completeness; subscribe_scan:false in
            # rtabmap_params.yaml means it isn't used. Lidar feeds Nav2 only.
            ('scan', '/scan'),
            ('map', '/map'),
        ],
        arguments=rtabmap_args)

    # ============ RViz2 ============
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'rviz.rviz')],
        parameters=[{'use_sim_time': False}],
        output='screen')

    return [
        zed,
        ekf_node,
        rgbd_sync_node,
        rtabmap_node,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Run RTAB-Map in localization mode (loads existing db). '
                        'If false, runs in mapping mode (deletes previous db).'),
        DeclareLaunchArgument(
            'db_path',
            default_value=os.path.join(os.path.expanduser('~'), '.ros', 'rtabmap.db'),
            description='Path to RTAB-Map database file'),
        OpaqueFunction(function=launch_setup),
    ])
