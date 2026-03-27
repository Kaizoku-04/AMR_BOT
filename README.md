<p align="center">
  <h1 align="center">🤖 Open AMR</h1>
  <p align="center">
    <strong>An open-source Autonomous Mobile Robot platform built on ROS 2</strong>
  </p>
  <p align="center">
    <a href="#features">Features</a> •
    <a href="#architecture">Architecture</a> •
    <a href="#getting-started">Getting Started</a> •
    <a href="#usage">Usage</a> •
    <a href="#packages">Packages</a> •
    <a href="#contributing">Contributing</a>
  </p>
</p>

---

## Overview

**Open AMR** is a modular, ROS 2-based autonomous mobile robot designed for indoor navigation, mapping, and docking. It features a differential-drive platform with LiDAR, IMU, and RGBD camera sensors, running the full Nav2 navigation stack with SLAM, EKF-fused localization, and AprilTag-based autonomous docking.

The project targets **Gazebo Sim (Harmonic)** for simulation and is built with extensibility in mind — every subsystem (description, localization, navigation, docking) is cleanly separated into its own ROS 2 package.

## Features

- **Differential Drive** — Two-wheel differential drive with four passive caster wheels
- **Sensor Suite** — RPLidar (2D LiDAR), Intel RealSense D435 (RGBD), IMU
- **SLAM & Mapping** — Online SLAM via `slam_toolbox` with loop closure
- **Localization** — EKF sensor fusion (wheel odometry + IMU) via `robot_localization`, AMCL for map-based localization
- **Autonomous Navigation** — Full Nav2 stack with MPPI and Regulated Pure Pursuit controllers
- **Autonomous Docking** — AprilTag-based dock detection with Nav2 docking server
- **Assisted Teleoperation** — Obstacle-aware teleoperation using Nav2's AssistedTeleop behavior
- **Simulation** — Pre-built Gazebo worlds (cafe, house, empty) with ros2_control integration
- **Custom Messages** — `TimedRotation` action and `SetCleaningState` service for application-layer tasks

## Architecture

```
open_amr_ws/src/open_amr/
├── open_amr/                    # Metapackage (depends on all sub-packages)
├── open_amr_description/        # URDF/Xacro, meshes, ros2_control config
├── open_amr_gazebo/             # Gazebo worlds, models, ros_gz_bridge config
├── open_amr_bringup/            # Top-level launch files & shell scripts
├── open_amr_localization/       # EKF configuration & launch
├── open_amr_navigation/         # Nav2 params, maps, nav scripts, cmd_vel relay
├── open_amr_docking/            # AprilTag dock detection & pose publisher
├── open_amr_msgs/               # Custom action/service definitions
├── open_amr_system_tests/       # Test nodes (square drive, parameter demo, etc.)
└── diff_drive_controller/       # Custom diff drive controller (ros2_control)
```

### System Diagram

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           Gazebo Sim                                    │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────────────┐   │
│  │  LiDAR   │  │   IMU    │  │  RGBD    │  │  Diff Drive (joints) │   │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └──────────┬───────────┘   │
└───────┼──────────────┼─────────────┼───────────────────┼───────────────┘
        │ ros_gz_bridge│             │                   │ gz_ros2_control
   ┌────▼────┐   ┌─────▼────┐  ┌────▼─────┐   ┌────────▼──────────┐
   │  /scan  │   │ /imu/data│  │ /cam_1/* │   │ diff_drive_ctrl   │
   └────┬────┘   └────┬─────┘  └────┬─────┘   │  ├─ /odom         │
        │              │             │          │  └─ /cmd_vel      │
        │         ┌────▼─────────────┘          └────────┬──────────┘
        │         │  EKF (robot_localization)             │
        │         │  /odometry/filtered                   │
        │         └──────────┬───────────────────────────►│
        │                    │                            │
   ┌────▼────────────────────▼────────────────────────────▼──┐
   │                    Nav2 Stack                            │
   │  AMCL │ Planner │ Controller │ Behaviors │ Costmaps     │
   └─────────────────────────────────────────────────────────┘
```

## Robot Specifications

| Parameter | Value |
|---|---|
| **Drive Type** | Differential drive |
| **Dimensions (L × W × H)** | 806 × 582 × 304 mm |
| **Weight** | ~15 kg (base) |
| **Wheel Radius** | 100 mm |
| **Wheel Separation** | 400 mm |
| **Max Linear Velocity** | 0.5 m/s |
| **Max Angular Velocity** | 2.5 rad/s |
| **LiDAR** | RPLidar (360°, 30m range, 720 samples) |
| **Camera** | Intel RealSense D435 (RGBD, 424×240) |
| **IMU** | 15 Hz update rate |

## Getting Started

### Prerequisites

- **Ubuntu 24.04** (Noble Numbat)
- **ROS 2 Jazzy** (or compatible distribution)
- **Gazebo Harmonic** (gz-sim)
- **Nav2** navigation stack
- **slam_toolbox**
- **robot_localization**

### Installation

1. **Install ROS 2 and Gazebo** — Follow the official [ROS 2 installation guide](https://docs.ros.org/en/jazzy/Installation.html)

2. **Install system dependencies:**
   ```bash
   sudo apt install -y \
     ros-${ROS_DISTRO}-nav2-bringup \
     ros-${ROS_DISTRO}-navigation2 \
     ros-${ROS_DISTRO}-slam-toolbox \
     ros-${ROS_DISTRO}-robot-localization \
     ros-${ROS_DISTRO}-ros-gz \
     ros-${ROS_DISTRO}-gz-ros2-control \
     ros-${ROS_DISTRO}-ros2-controllers \
     ros-${ROS_DISTRO}-teleop-twist-keyboard \
     ros-${ROS_DISTRO}-apriltag-ros \
     ros-${ROS_DISTRO}-image-proc \
     ros-${ROS_DISTRO}-xacro \
     ros-${ROS_DISTRO}-joint-state-publisher-gui \
     ros-${ROS_DISTRO}-rviz2
   ```

3. **Clone and build the workspace:**
   ```bash
   mkdir -p ~/open_amr_ws/src && cd ~/open_amr_ws/src
   git clone https://github.com/Kaizoku-04/AMR_BOT.git open_amr
   cd ~/open_amr_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```

## Usage

### Visualize the Robot Model

Launch the robot description in RViz to inspect the URDF:

```bash
ros2 launch open_amr_description robot_state_publisher.launch.py \
    use_gazebo:=false use_rviz:=true jsp_gui:=true
```

### Launch Gazebo Simulation

Start the robot in a Gazebo world with ros2_control:

```bash
# Using the convenience script (cafe world):
bash src/open_amr/open_amr_bringup/scripts/open_amr_gazebo.sh

# Or directly via launch file:
ros2 launch open_amr_gazebo open_amr.gazebo.launch.py \
    world_file:=cafe.world \
    use_rviz:=true \
    x:=0.0 y:=0.0 z:=0.20
```

**Available worlds:** `empty.world`, `cafe.world`, `house.world`, `pick_and_place_demo.world`

### Teleoperate the Robot

In a separate terminal:

```bash
ros2 launch open_amr_bringup teleop.launch.py
```

### Launch Full Navigation Stack

Start Gazebo + Nav2 + SLAM/Localization + EKF + Docking:

```bash
# With SLAM (for mapping):
bash src/open_amr/open_amr_bringup/scripts/open_amr_navigation.sh slam

# With AMCL (using a pre-built map):
bash src/open_amr/open_amr_bringup/scripts/open_amr_navigation.sh

# Or directly via launch file:
ros2 launch open_amr_bringup open_amr_navigation.launch.py \
    world_file:=cafe.world \
    slam:=True \
    z:=0.20
```

### Navigate to a Specific Pose

Use the interactive table selector (cafe world):

```bash
ros2 run open_amr_navigation test_nav_to_pose.py
```

Or publish a goal pose directly:

```bash
ros2 topic pub /goal_pose/goal geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: -0.96, y: -0.92, z: 0.0}, orientation: {w: 1.0}}}"
```

### Monitor Navigation Status

```bash
# Goal status:
ros2 topic echo /goal_pose/status

# Estimated time of arrival:
ros2 topic echo /goal_pose/eta
```

## Packages

### `open_amr_description`
Robot model definition using URDF/Xacro with modular macros for the base chassis, differential wheels, caster wheels, LiDAR, IMU, and RGBD camera. Includes ros2_control hardware interface definitions and controller configuration.

### `open_amr_gazebo`
Gazebo Sim integration with pre-built simulation worlds, 90+ 3D models (AWS RoboMaker assets), ros_gz_bridge configuration for sensor/command topic bridging, and robot spawning.

### `open_amr_bringup`
Top-level launch files that orchestrate the full system — Gazebo simulation, ros2_control controller loading, teleop, and the complete navigation stack. Includes convenience shell scripts.

### `open_amr_localization`
Extended Kalman Filter (EKF) configuration using `robot_localization`. Fuses wheel odometry (linear velocity, yaw rate) with IMU data (yaw rate, linear acceleration) for robust state estimation.

### `open_amr_navigation`
Nav2 parameter files (MPPI and Regulated Pure Pursuit controllers), pre-built maps, navigation scripts (`nav_to_pose.py`, `assisted_teleoperation.py`), and the `cmd_vel_relay` node that converts `Twist` to `TwistStamped`.

### `open_amr_docking`
AprilTag-based autonomous docking using Nav2's docking server. Includes a `detected_dock_pose_publisher` C++ node that listens for AprilTag TF transforms and publishes them as `PoseStamped` for the docking controller.

### `open_amr_msgs`
Custom ROS 2 interface definitions:
- **`TimedRotation.action`** — Rotate the robot at a specified angular velocity for a given duration
- **`SetCleaningState.srv`** — Start/stop cleaning operations

### `open_amr_system_tests`
Test and demo nodes: square-driving controller, parameter demonstration, timed rotation action server/client, and cleaning state service server.

### `diff_drive_controller`
Custom differential drive controller plugin for ros2_control with odometry computation and speed limiting.

## Key Topics

| Topic | Type | Description |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | 2D LiDAR scans |
| `/imu/data` | `sensor_msgs/Imu` | IMU readings |
| `/cam_1/color/image_raw` | `sensor_msgs/Image` | RGB camera image |
| `/cam_1/depth/color/points` | `sensor_msgs/PointCloud2` | Depth point cloud |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (from Nav2) |
| `/diff_drive_controller/cmd_vel` | `geometry_msgs/TwistStamped` | Stamped velocity commands (to controller) |
| `/diff_drive_controller/odom` | `nav_msgs/Odometry` | Wheel odometry |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF-fused odometry |
| `/goal_pose/goal` | `geometry_msgs/PoseStamped` | Navigation goal |
| `/goal_pose/status` | `std_msgs/String` | Navigation status |
| `/goal_pose/eta` | `std_msgs/String` | Estimated time of arrival |
| `/detected_dock_pose` | `geometry_msgs/PoseStamped` | Detected dock position |

## TF Tree

```
map
 └── odom                    (published by AMCL or slam_toolbox)
      └── base_footprint     (published by EKF / diff_drive_controller)
           └── base_link
                ├── left_wheel_link
                ├── right_wheel_link
                ├── front_left_caster
                ├── front_right_caster
                ├── rear_left_caster
                ├── rear_right_caster
                ├── laser_frame
                ├── imu_link
                └── cam_1_link
                     ├── cam_1_depth_frame
                     │    └── cam_1_depth_optical_frame
                     ├── cam_1_color_frame
                     │    └── cam_1_color_optical_frame
                     ├── cam_1_infra1_frame
                     │    └── cam_1_infra1_optical_frame
                     └── cam_1_infra2_frame
                          └── cam_1_infra2_optical_frame
```

## Configuration

### Navigation Controllers

Two Nav2 controller configurations are provided:

| Config File | Controller | Best For |
|---|---|---|
| `open_amr_nav2_default_params.yaml` | MPPI Controller | Dynamic environments, smoother paths |
| `open_amr_nav2_regulated_pure_pursuit_controller.yaml` | Regulated Pure Pursuit + Rotation Shim | Precise path following, structured environments |

Switch controllers by passing the `nav2_params_file` argument:

```bash
ros2 launch open_amr_bringup open_amr_navigation.launch.py \
    nav2_params_file:=/path/to/open_amr_nav2_regulated_pure_pursuit_controller.yaml
```

### EKF Sensor Fusion

The EKF is configured in `open_amr_localization/config/ekf.yaml`:
- **From wheel odometry:** linear velocity (x), yaw rate
- **From IMU:** yaw rate, linear acceleration (x)
- **Mode:** 2D (planar)
- **Output frame:** `odom` → `base_footprint`

## Project Structure

```
open_amr/
├── open_amr/                          # Metapackage
│   ├── CMakeLists.txt
│   └── package.xml
├── open_amr_description/
│   ├── config/                        # ros2_controllers.yaml
│   ├── launch/                        # robot_state_publisher.launch.py
│   ├── meshes/                        # STL/DAE mesh files
│   ├── rviz/                          # RViz config
│   └── urdf/
│       ├── control/                   # ros2_control xacro macros
│       ├── mech/                      # Base, wheel xacro macros
│       ├── robots/                    # Top-level robot xacro
│       └── sensors/                   # LiDAR, IMU, camera xacro macros
├── open_amr_gazebo/
│   ├── config/                        # ros_gz_bridge.yaml
│   ├── launch/                        # open_amr.gazebo.launch.py
│   ├── models/                        # 90+ Gazebo 3D models
│   ├── rviz/                          # Gazebo-specific RViz config
│   └── worlds/                        # .world files (cafe, house, empty)
├── open_amr_bringup/
│   ├── launch/                        # Full system launch files
│   └── scripts/                       # Convenience shell scripts
├── open_amr_localization/
│   ├── config/                        # ekf.yaml
│   └── launch/                        # ekf_gazebo.launch.py
├── open_amr_navigation/
│   ├── config/                        # Nav2 parameter files
│   ├── maps/                          # Pre-built map files (.pgm + .yaml)
│   ├── open_amr_navigation/           # Python module (PoseStampedGenerator)
│   ├── rviz/                          # Nav2 RViz config
│   ├── scripts/                       # Navigation Python scripts
│   └── src/                           # cmd_vel_relay C++ node
├── open_amr_docking/
│   ├── config/                        # AprilTag & dock database configs
│   ├── launch/                        # Docking launch file
│   └── src/                           # Dock pose publisher C++ node
├── open_amr_msgs/
│   ├── action/                        # TimedRotation.action
│   └── srv/                           # SetCleaningState.srv
├── open_amr_system_tests/
│   ├── config/                        # Test parameter files
│   ├── launch/                        # Test launch files
│   └── src/                           # Test C++ nodes
└── diff_drive_controller/             # Custom ros2_control plugin
    ├── include/                        # Header files
    ├── src/                            # Implementation
    └── test/                           # Unit tests
```

## Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/my-feature`)
3. Commit your changes (`git commit -m 'Add my feature'`)
4. Push to the branch (`git push origin feature/my-feature`)
5. Open a Pull Request

### Development Setup

```bash
# Build with debug symbols:
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Run tests:
colcon test
colcon test-result --verbose
```

## License

This project is licensed under the **BSD-3-Clause License** — see the [LICENSE](LICENSE) files in each package for details.

## Acknowledgments

- [Nav2](https://navigation.ros.org/) — ROS 2 Navigation Framework
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) — SLAM for ROS 2
- [robot_localization](https://github.com/cra-ros-pkg/robot_localization) — EKF/UKF state estimation
- [ros2_control](https://github.com/ros-controls/ros2_control) — Hardware abstraction and controllers
- [Gazebo Sim](https://gazebosim.org/) — Robot simulation
- [AWS RoboMaker](https://github.com/aws-robotics) — Simulation world models
