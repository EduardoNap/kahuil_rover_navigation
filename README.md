# Kahuil Rover: Autonomous Navigation (ROS 2)

Autonomous indoor navigation stack for a rescue robot, built for **RoboCup Rescue 2026**. It runs on a real 4-wheel rover (NVIDIA Jetson, ROS 2 Humble). The rover is used to validate the full perception → SLAM → planning pipeline before porting it to the tracked rescue robot.

The robot builds a map with **RTAB-Map** (ZED 2i RGB-D + visual-inertial odometry), localizes in it, and navigates to goals with **Nav2**, avoiding obstacles in 3D. A **frontier-based explorer** can drive it autonomously through unknown areas.

<!-- Add a photo/GIF of the rover and an RViz screenshot of the map here -->

---

## Highlights

- **Visual SLAM with RTAB-Map**: builds the 2D occupancy grid from ZED depth, with ground/obstacle height bands tuned for the 60 cm walls of the rescue course and noise filtering for clean walls. Supports mapping and localization modes.
- **Sensor fusion (robot_localization EKF)**: takes x, y from ZED visual-inertial odometry and yaw from the ZED IMU, so the IMU is not counted twice. The EKF publishes the `odom → base_footprint` transform.
- **Nav2 with 3D obstacle awareness**: a voxel layer fed by the ZED point cloud catches overhangs and low obstacles that a 2D lidar misses. The RPLidar A1 feeds the local obstacle layer. Uses the Regulated Pure Pursuit controller and the NavFn planner.
- **Frontier exploration**: a custom explorer node with multi-criteria frontier scoring and continuous wall monitoring sends goals to Nav2. It comes with a Flask + WebSocket web dashboard for live monitoring.
- **Real hardware integration via ros2_control**: an ESP32 running the `ros_arduino_bridge` protocol drives a Sabertooth motor driver with encoder feedback and 30 Hz PID. The serial layer was patched to stop ESP32 auto-reset loops.
- **Robot descriptions**: URDF/xacro for the rover (with ZED and lidar mounts from real measurements) and the tracked rescue robot model, plus a Gazebo rescue-arena world.

## System architecture

```
 ZED 2i ──► /zed/zed_node/odom (VIO) ─┐
        └─► IMU (yaw) ────────────────┼─► EKF ──► TF odom→base_footprint ──┐
                                      │                                    │
 ZED 2i ──► RGB-D (rgbd_sync) ───────────────────────► RTAB-Map ◄──────────┘
                                                          │
                                                          ├─► /map
                                                          └─► TF map→odom
 ZED 2i ──► point cloud ──► Nav2 voxel layer ─┐
 RPLidar ─► /scan ────────► Nav2 obstacle layer ─┼─► Nav2 (RPP + NavFn) ──► twist_mux ──► diff_cont
 /map ────────────────────► Nav2 static layer ─┘         ▲                                │
                                                        │                        ros2_control
 Frontier explorer ── goals ────────────────────────────┘                               │
                                                                        ESP32 ─► Sabertooth ─► motors
```

**TF tree:** `map ─(RTAB-Map)→ odom ─(EKF)→ base_footprint ─(URDF)→ base_link → {wheels, zed_camera_link, lidar_link}`

## Hardware

| Component | Role |
|---|---|
| NVIDIA Jetson (ARM64) | Onboard computer, ROS 2 Humble |
| ZED 2i stereo camera | RGB-D, visual-inertial odometry, IMU, point cloud |
| RPLidar A1 | 2D scan for local obstacle avoidance |
| ESP32 + Sabertooth | Motor control and encoder reading (PID at 30 Hz) |
| 4-wheel skid-steer rover | 2 motors, 2 encoders, front/back wheels coupled |

## Repository structure

```
kahuil_nav/
├── description/   # URDF/xacro: rover, tracked robot, ZED, lidar, ros2_control
├── config/        # rtabmap, ekf, nav2, controllers, twist_mux, explore params
├── launch/        # launch_robot, rtabmap_slam, nav2, exploration, teleop, sim
├── scripts/       # frontier_explorer.py, exploration_dashboard.py
├── rviz/          # RViz configs
└── worlds/        # Gazebo rescue arena
diffdrive_arduino/ # ros2_control hardware interface (third-party, patched; see Credits)
```

## Getting started

**Requirements:** Ubuntu 22.04, ROS 2 Humble, the ZED SDK, and these packages:

```bash
sudo apt install ros-humble-rtabmap-ros ros-humble-zed-* ros-humble-nav2-bringup \
  ros-humble-robot-localization ros-humble-twist-mux ros-humble-ros2-control \
  ros-humble-ros2-controllers ros-humble-depthimage-to-laserscan
# RPLidar driver: https://github.com/Slamtec/sllidar_ros2
```

**Build:**

```bash
mkdir -p ~/kahuil_ws/src && cd ~/kahuil_ws/src
git clone <this-repo> .
cd ~/kahuil_ws && colcon build --symlink-install
source install/setup.bash
```

**Run** (three terminals):

```bash
ros2 launch kahuil_nav launch_robot.launch.py     # hardware, ros2_control, lidar
ros2 launch kahuil_nav rtabmap_slam.launch.py     # ZED + RTAB-Map (mapping)
ros2 launch kahuil_nav nav2.launch.py             # Nav2
```

- Localization on a saved map: `ros2 launch kahuil_nav rtabmap_slam.launch.py localization:=true`
- Autonomous exploration: `ros2 launch kahuil_nav exploration.launch.py`
- In RViz, set the Fixed Frame to `map` and send goals with **2D Goal Pose**.

## Roadmap

- [x] Phase 1: slam_toolbox + wheel odometry + lidar + Nav2
- [x] Phase 2: RTAB-Map + ZED VIO/IMU fusion + Nav2
- [x] Phase 3: 3D voxel costmap layer from the ZED point cloud
- [ ] Localization-mode validation on a fixed map
- [ ] OpenManipulator-X arm integration (single unified controller_manager)
- [ ] Port the full stack to the tracked rescue robot

## Credits

- [`diffdrive_arduino`](https://github.com/joshnewans/diffdrive_arduino) by Josh Newans (Apache-2.0), included with a local patch to the serial connection (DTR/RTS handling for ESP32).
- Robot structure inspired by [`articubot_one`](https://github.com/joshnewans/articubot_one).
- Built on [RTAB-Map](https://github.com/introlab/rtabmap_ros), [Nav2](https://nav2.org), [robot_localization](https://github.com/cra-ros-pkg/robot_localization), and the [ZED ROS 2 wrapper](https://github.com/stereolabs/zed-ros2-wrapper).

## Author

Eduardo Rey García Velasco
