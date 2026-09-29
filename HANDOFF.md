# Kahuil Rover (RTAB-Map variant) — Nav2 Bring-up Handoff

_Last updated: 2026-05-23_

## Project context

- **Robot**: 4-wheel rover (`q-mars-1` from `rover_sim`) — 2 motors, 2 encoders, parallel-coupled pairs (front + back on each side rigidly linked). Stand-in to validate the navigation stack before porting to the tracked rescue robot (`kahuil_new`).
- **Platform**: ROS 2 **Humble** on Jetson (ARM64, Linux 5.15 Tegra). Real-robot only, no Gazebo / sim.
- **Sensors**: ZED 2i (mounted, wired into ROS — provides RGB-D + visual odometry) + RPLidar A1 (mounted on top of ZED, working).
- **Hardware interface**: ESP32 running [joshnewans/ros_arduino_bridge](https://github.com/joshnewans/ros_arduino_bridge) protocol, controls a Sabertooth motor driver. Encoders via ESP32Encoder library on pins 26/25 (R) and 33/32 (L). PID at 30 Hz.
- **Workspace**: `~/Desktop/kahuil_new_rover_rtab/kahuil_ws/` (sibling clone of `kahuil_new_rover/`, dedicated to the RTAB-Map stack).
- **Goal**: RoboCup Rescue 2026 — ZED VIO + RTAB-Map + Nav2 on the tracked robot. This rover validates that path end-to-end.

---

## What's different from `kahuil_new_rover`

| | `kahuil_new_rover` | `kahuil_new_rover_rtab` (this project) |
|---|---|---|
| SLAM | slam_toolbox (lidar + wheel odom) | **RTAB-Map** (ZED RGB-D, ZED VIO) |
| `odom → base_footprint` TF | `diff_cont` (wheel odom) | **`robot_localization` EKF** (fed by ZED VIO) |
| `map → odom` TF | slam_toolbox | **rtabmap_slam** |
| Lidar role | SLAM + Nav2 local costmap | **Nav2 local costmap only** |
| Bring-up | `launch_robot` + `online_async_slam` + `nav2` | `launch_robot` + **`rtabmap_slam`** + `nav2` |

slam_toolbox launches and configs have been removed from this project. If you need slam_toolbox, work in `kahuil_new_rover/`.

---

## Workspaces in use

| Path | Contents |
|---|---|
| `~/ros2_ws/install/` | `sllidar_ros2` (Slamtec official RPLidar driver) |
| `~/Desktop/kahuil_new_rover_rtab/kahuil_ws/` | `kahuil_nav` (description + launches + configs), `diffdrive_arduino` (humble branch, patched) |

System packages required (apt): `ros-humble-rtabmap-*`, `ros-humble-zed-*`, `ros-humble-depthimage-to-laserscan`, `ros-humble-nav2-bringup`.

Every shell needs both sourced, in this order:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/Desktop/kahuil_new_rover_rtab/kahuil_ws/install/setup.bash
```

---

## Physical robot parameters (real measurements)

| | |
|---|---|
| Wheel radius | 0.065 m (6.5 cm) |
| Wheel width | 0.05 m |
| Wheel separation (L↔R, **geometric**, URDF) | 0.53 m |
| Wheel separation (**diff_cont calibration**, `my_controllers.yaml`) | **0.90 m** |
| Wheelbase (F↔B) | 0.43 m |
| Chassis | 0.40 × 0.37 × 0.03 m, 6 kg |
| Chassis bottom above wheel-axle plane | 0.115 m |
| ZED position from base_link | x=0.15, y=0, z=0.17 |
| Lidar position from base_link | x=0.15, y=0, z=0.25 (scan plane) |
| Encoder counts/rev | 7338 |
| Motor port | `/dev/ttyUSB0` (FTDI on ESP32) |
| Lidar port | `/dev/ttyCH341USB0` (CH341 USB-serial, different kernel driver) |

> **wheel_separation 0.90 override.** URDF uses the geometric 0.53 m, but `diff_cont` in `my_controllers.yaml` is set to 0.90 m as a calibration override to compensate for skid-steer slip on this 4-wheel rover. Carried over from the slam_toolbox project. With ZED VIO as the odom source the override no longer affects the SLAM TF — but `/odom` (wheel) is still used by Nav2 as a velocity feedback / diagnostic, so keep it correct.

---

## File map (kahuil_nav)

```
src/kahuil_nav/
├── CMakeLists.txt
├── description/
│   ├── rover.urdf.xacro              ← root URDF, includes zed.xacro + lidar.xacro + rover_ros2_control.xacro
│   ├── zed.xacro                     ← ZED 2i mount (fixed joint, x=0.15 z=0.17)
│   ├── lidar.xacro                   ← RPLidar A1, lidar_link at scan plane (z=0.25)
│   ├── rover_ros2_control.xacro      ← diffdrive_arduino plugin block
│   └── inertial_macros.xacro         ← inertia helpers
├── config/
│   ├── my_controllers.yaml           ← diff_cont (wheel_separation:0.90); enable_odom_tf:false
│   ├── twist_mux.yaml                ← cmd_vel_joy=100, cmd_vel_nav=10 → /cmd_vel
│   ├── nav2_params.yaml              ← Nav2 (RPP controller, NavFn planner)
│   ├── ekf_params.yaml               ← robot_localization EKF (ZED VIO → odom→base_footprint TF)
│   ├── rtabmap_params.yaml           ← RTAB-Map mapping mode (depth-only Grid)
│   └── rtabmap_localization_params.yaml  ← RTAB-Map localization mode
├── launch/
│   ├── rsp.launch.py                 ← robot_state_publisher; arg robot_model in {rover, tracked}
│   ├── launch_robot.launch.py        ← RSP + ros2_control + diff_cont + joint_broad + twist_mux + sllidar
│   ├── rtabmap_slam.launch.py        ← ZED wrapper + rgbd_sync + rtabmap (mapping OR localization)
│   ├── nav2.launch.py                ← wraps nav2_bringup/navigation_launch.py (no AMCL)
│   ├── exploration.launch.py
│   ├── teleop.launch.py
│   └── sim.launch.py                 ← inherited from tracked robot, not used
└── meshes/                           ← STL files (not used now we're on primitives)
```

---

## TF tree

```
map ──(rtabmap)──> odom ──(EKF)──> base_footprint ──(URDF)──> base_link
                                                              ├─> left_front_wheel_link  (continuous, controlled)
                                                              ├─> right_front_wheel_link (continuous, controlled)
                                                              ├─> left_back_wheel_link   (mimic)
                                                              ├─> right_back_wheel_link  (mimic)
                                                              ├─> zed_camera_link        (fixed) ──(ZED RSP)──> zed_camera_center → optical frames
                                                              └─> lidar_link             (fixed; /scan frame)
```

Why EKF? `zed_wrapper` 5.x hardcodes its base frame to `zed_camera_link` — there's no parameter to make it publish `odom → base_footprint` directly. Instead:
- `zed_wrapper`: `publish_tf: false` (does NOT publish odom→zed_camera_link TF — would collide with URDF's `base_link → zed_camera_link`). The `/zed/zed_node/odom` *topic* still publishes.
- `robot_localization` EKF: subscribes to `/zed/zed_node/odom`, uses URDF static chain to compute, publishes `odom → base_footprint`. Output also on topic `/odometry/filtered`.
- `diff_cont`: `enable_odom_tf: false` (in `my_controllers.yaml`) — wheel odom topic only, no TF.

---

## ros2_control hardware

`rover_ros2_control.xacro` declares **2 joints only** (`left_front_wheel_joint`, `right_front_wheel_joint`) because back wheels are mechanically coupled to the fronts. Back wheels are URDF mimic joints for RViz visualization only. State-interface order must be **position before velocity** (humble plugin requires it).

```xml
<plugin>diffdrive_arduino/DiffDriveArduinoHardware</plugin>
<param name="device">/dev/ttyUSB0</param>
<param name="baud_rate">57600</param>
<param name="timeout_ms">1000</param>
<param name="enc_counts_per_rev">7338</param>
```

---

## diffdrive_arduino patch (kept)

`src/diffdrive_arduino/hardware/include/diffdrive_arduino/arduino_comms.hpp` was patched to deassert DTR/RTS + wait 2 s on open, to prevent ESP32 auto-reset loops from the libserial DTR pulse:

```cpp
void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms) {
  timeout_ms_ = timeout_ms;
  serial_conn_.Open(serial_device);
  serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  serial_conn_.SetDTR(false);
  serial_conn_.SetRTS(false);
  std::this_thread::sleep_for(std::chrono::seconds(2));
  serial_conn_.FlushIOBuffers();
}
```

Patch is local to the clone; will be lost on `git pull`.

---

## ESP32 firmware fix (CRITICAL — done)

Original bug: `Serial.readStringUntil('\n')` in the firmware loop. The plugin sends `e\r` (CR only), so commands were never recognized. Fix: change `'\n'` to `'\r'`.

Side-effect: miniterm now needs `--eol CR`:
```bash
python3 -m serial.tools.miniterm /dev/ttyUSB0 57600 --eol CR
```

---

## Bring-up sequence

Three terminals, all with both workspaces sourced.

| Terminal | Command |
|---|---|
| A | `ros2 launch kahuil_nav launch_robot.launch.py` |
| B | `ros2 launch kahuil_nav rtabmap_slam.launch.py` |
| C | `ros2 launch kahuil_nav nav2.launch.py` |

Then RViz, Fixed Frame = `map`, add Map / LaserScan / RobotModel / TF / Costmap displays, send goals via 2D Goal Pose.

### Mapping vs localization

- **Mapping** (default): `ros2 launch kahuil_nav rtabmap_slam.launch.py` — runs with `-d` so the existing db at `~/.ros/rtabmap.db` is wiped first.
- **Localization**: `ros2 launch kahuil_nav rtabmap_slam.launch.py localization:=true` — keeps the db, loads it into memory, no new nodes added. Set 2D Pose Estimate in RViz if it doesn't auto-relocalize.
- **Custom db path**: `db_path:=/path/to/your.db` works in either mode.

---

## What feeds what

```
/zed/zed_node/odom ──> EKF ──> /odometry/filtered  +  TF odom→base_footprint
                                       │
                                       └──> rtabmap (subscribed as `odom`)
/rgbd_image (rgbd_sync) ──────────────────> rtabmap (frame_id:base_footprint, Grid/FromDepth:true)
                                                        │
                                                        └──> /map  +  TF map→odom

/scan (sllidar, frame lidar_link) ──> Nav2 local_costmap obstacle_layer
                                      (RTAB-Map does NOT subscribe — depth only)

/odom (diff_cont wheel) ──> Nav2 controller server (velocity feedback / diagnostics)
                            NOT a TF publisher
```

---

## Current nav2 tuning (in `config/nav2_params.yaml`)

Inherited from the slam_toolbox project (`kahuil_new_rover`); no RTAB-specific tuning yet.

| Setting | Value | Notes |
|---|---|---|
| `footprint` (local + global costmap) | `[[0.30, 0.30], [0.30, -0.30], [-0.30, -0.30], [-0.30, 0.30]]` | 60×60 cm rectangle, includes safety margin |
| `inflation_radius` (both costmaps) | 0.50 m | |
| Max angular velocity (controller, velocity_smoother) | ±0.8 rad/s | |
| `movement_time_allowance` (behavior_server) | 40.0 s | |
| Controller | RPP (Regulated Pure Pursuit) | |
| Planner | NavFn | |

Layers:
- `local_costmap`: obstacle_layer (lidar /scan) + inflation_layer (rolling)
- `global_costmap`: static_layer (from /map) + obstacle_layer + inflation_layer

---

## Phase roadmap

```
Phase 1  ✅ DONE      slam_toolbox + wheel odom + LiDAR + Nav2  (in kahuil_new_rover, archived here)
Phase 2  ◐ NOW        RTAB-Map + ZED VIO + LiDAR (Nav2 only) + Nav2  ← active stack
Phase 2.5 ◐ NEXT      Save db, switch to localization mode, validate Nav2 against fixed map
Phase 3  ◐ LATER      Add voxel_layer to Nav2 costmaps fed by /zed/zed_node/point_cloud/cloud_registered
                      (3D obstacle awareness for overhangs / tables / low obstacles)
Phase 4              Port the whole stack to the tracked robot in ~/Desktop/kahuil_new/
```

---

## Verification checklist (Phase 2)

After `colcon build --symlink-install` in `~/Desktop/kahuil_new_rover_rtab/kahuil_ws/`:

1. **TF chain**: `ros2 run tf2_tools view_frames` → `map → odom → base_footprint → base_link → {wheels, zed_camera_link, lidar_link}`.
2. **Topic rates**: `/zed/zed_node/odom` ≥15 Hz, `/scan` ~10 Hz, `/odom` 50 Hz, `/map` updating.
3. **No duplicate TF publisher**: `ros2 run tf2_ros tf2_echo odom base_footprint` resolves cleanly without flicker. If it flickers, `enable_odom_tf` is still true somewhere.
4. **Drive test**: teleop ~2 m forward / back, then rotate 360°. Map should grow; lidar in RViz must NOT rotate with the robot.
5. **Save + relocalize**: kill terminal B, relaunch with `localization:=true`, set 2D Pose Estimate, verify it tracks.
6. **Nav goal**: 2D Goal Pose in RViz → robot reaches goal, costmaps stay stable.

Most likely failure points (in priority order): TF chain → ZED `base_frame` parameter → RTAB-Map `frame_id` alignment.

---

## Pending / TODO

- [ ] **OpenManipulator-X integration** — arm uses ros2_control too; launching both robot.launch + arm causes `controller_manager` singleton conflict. Recommended fix: merge URDFs and run one unified controller_manager. **Deferred by user.**
- [ ] Add `voxel_layer` to nav2 costmaps fed by ZED point cloud (3D obstacle awareness)
- [ ] Tune `Grid/MaxObstacleHeight` / `Grid/MaxGroundHeight` for the actual environment
- [ ] Decide whether to fuse wheel odom into ZED VIO via robot_localization EKF (currently rejected — re-evaluate if VIO drifts in featureless corridors)
- [ ] Re-tune Nav2 (`failure_tolerance`, `transform_tolerance`) for RTAB-Map's slower map-update cadence vs slam_toolbox
- [ ] Add IMU joint to `rover.urdf.xacro` (position TBD)
- [ ] udev rules so ESP32 always gets `/dev/ttyESP32` and the lidar `/dev/ttyLIDAR`
- [ ] Port the whole stack to the tracked robot in `~/Desktop/kahuil_new/`

---

## Reference

- articubot_one (humble): https://github.com/joshnewans/articubot_one/tree/humble
- diffdrive_arduino (humble): https://github.com/joshnewans/diffdrive_arduino/tree/humble
- rtabmap_ros (humble): https://github.com/introlab/rtabmap_ros/tree/humble
- zed-ros2-wrapper: https://github.com/stereolabs/zed-ros2-wrapper
- ros_arduino_bridge protocol: `e` = read encoders, `m L R` = motor cmd, `o L R` = open-loop, `u Kp:Kd:Ki:Ko` = PID gains, `r` = reset encoders. Commands terminated with `\r`.
