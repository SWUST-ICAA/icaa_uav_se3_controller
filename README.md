# ICAA UAV SE3 Controller

[![ROS Version](https://img.shields.io/badge/ROS-Kinetic%20%7C%20Melodic%20%7C%20Noetic-blue.svg)](http://wiki.ros.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)](LICENSE)
[![Version](https://img.shields.io/badge/Version-1.0.0-brightgreen.svg)](https://github.com/SWUST-ICAA/icaa_uav_se3_controller/releases)

A ROS package implementing SE3 geometric controller for multi-rotor UAVs using PX4 platform.

## Overview

The `icaa_uav_se3_controller` is a nonlinear geometric controller based on SE(3) that provides accurate trajectory tracking for quadrotors and other multi-rotor platforms. It features:

- **SE3 Geometric Control**: Implements the control algorithm on the Special Euclidean group SE(3)
- **Path Following**: Subscribes to `nav_msgs/Path` messages for trajectory tracking
- **Auto Takeoff/Landing**: Automated takeoff and landing capabilities
- **Thrust Mapping**: Adaptive thrust model estimation
- **Integral Action**: World and body frame integral terms for disturbance rejection
- **Mass Estimation**: Online mass estimation for unknown payloads

## Prerequisites

- ROS Kinetic/Melodic/Noetic
- PX4 Autopilot
- MAVROS
- Eigen3

## Installation

1. Clone this repository into your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/SWUST-ICAA/icaa_uav_se3_controller.git
```

2. Build the package:
```bash
cd ~/catkin_ws
catkin_make
```

## Configuration

The controller parameters can be configured in `config/se3_controller_param.yaml`. Key parameters include:

- **SE3 Controller Gains**: Position, velocity, attitude, and attitude rate gains
- **Integral Gains**: World and body frame integral gains with saturation limits
- **Mass Estimator**: Gain and limits for online mass estimation
- **Constraints**: Tilt angle failsafe and throttle saturation limits

## Usage

### Basic Launch

To start the SE3 controller:
```bash
roslaunch icaa_uav_se3_controller run_se3_ctrl.launch
```

### Topics

**Subscribed Topics:**
- `/vins_fusion/imu_propagate` ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)): State estimation from VIO
- `/planning/pos_cmd_path` ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html)): Desired trajectory path
- `/mavros/state` ([mavros_msgs/State](http://docs.ros.org/api/mavros_msgs/html/msg/State.html)): FCU connection and arming state
- `/mavros/imu/data` ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)): IMU measurements
- `/mavros/rc/in` ([mavros_msgs/RCIn](http://docs.ros.org/api/mavros_msgs/html/msg/RCIn.html)): RC input (optional)
- `/mavros/battery` ([sensor_msgs/BatteryState](http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html)): Battery status

**Published Topics:**
- `/mavros/setpoint_raw/attitude` ([mavros_msgs/AttitudeTarget](http://docs.ros.org/api/mavros_msgs/html/msg/AttitudeTarget.html)): Attitude/thrust commands
- `/debugSE3ctrl` (icaa_uav_se3_controller/Px4ctrlDebug): Debug information
- `/traj_start_trigger` ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)): Trajectory start trigger

### Thrust Calibration

To calibrate the thrust model:
```bash
roslaunch icaa_uav_se3_controller thrust_calibrate.launch
```

## State Machine

The controller operates with the following states:

1. **MANUAL_CTRL**: Direct RC control
2. **AUTO_HOVER**: Autonomous hovering at current position
3. **CMD_CTRL**: Following trajectory commands
4. **AUTO_TAKEOFF**: Automated takeoff sequence
5. **AUTO_LAND**: Automated landing sequence

## Safety Features

- Tilt angle limiting with configurable failsafe
- Throttle saturation
- RC override capability
- Low battery warnings
- Automatic disarm on landing

## Authors

- ICAA UAV Team - Southwest University of Science and Technology
- Maintainer: nanwan2004@126.com

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Based on the geometric control theory for quadrotors
- Inspired by PX4CTRL and MRS UAV Controllers