# Tow Tractor ROS2 Simulation & Control

This repository contains a modular ROS2 workspace for simulating, describing, and controlling a tow tractor robot. It supports both hardware and simulation (Gazebo) modes, and is organized into several packages for description, control, hardware interface, and simulation integration.

## Workspace Structure

```
.
├── src/
│   ├── tow_tractor/                # Custom message definitions (ModelInfo.msg)
│   ├── tow_tractor_bringup/        # Launch files and overall system bringup
│   ├── tow_tractor_control/        # Controllers (diff drive, rear steer)
│   ├── tow_tractor_description/    # Robot description, URDF, meshes, model info publisher
│   ├── tow_tractor_hardware/       # Hardware interface nodes (sensors, actuators)
│   ├── tow_tractor_gazebo/         # Gazebo simulation resources (worlds, models)
│   └── tow_tractor_slam/          # SLAM and localization configurations
└── README.md
```

## Main Features

### Message Definitions (`tow_tractor`)
- Custom ROS2 messages including `ModelInfo.msg` for robot parameters

### System Bringup (`tow_tractor_bringup`)
- Main launch file for complete system startup
- Integration of hardware and simulation modes
- RViz configuration for visualization
- Gazebo-ROS bridge configurations

### Robot Control (`tow_tractor_control`)
- Differential drive controller
- Rear wheel steering controller
- Command velocity processing

### Robot Description (`tow_tractor_description`)
- URDF/Xacro files for multiple robot variants
- Model info publisher node
- Mesh files and visual assets
- Robot configuration parameters

### Hardware Interface (`tow_tractor_hardware`)
- Sensor receiver node for IMU and encoders
- Actuator sender node for motors
- Odometry publisher
- Serial communication with Arduino

### Gazebo Simulation (`tow_tractor_gazebo`)
- Custom world definitions
- Gazebo model resources
- Simulation plugins configuration
- Environment hooks for Gazebo paths

### SLAM & Navigation (`tow_tractor_slam`)
- Integration with SLAM Toolbox
- EKF configuration for sensor fusion
- Navigation stack setup

## How to Build

From the root of your workspace:

```sh
colcon build --symlink-install
source install/setup.bash
```

## How to Launch

To bring up the full simulation (Gazebo + robot + control):

```sh
ros2 launch tow_tractor_bringup tow_tractor_bringup.launch.py
```

Launch arguments:
- `use_gazebo` (default: `true`): Toggle simulation mode
- `rviz` (default: `true`): Toggle visualization
- `robot_control` (options: `diff_drive`, `rear_steer`): Select control mode

## Hardware Mode

To run with real hardware:

```sh
ros2 launch tow_tractor_bringup tow_tractor_bringup.launch.py use_gazebo:=false
```

## Key Launch Arguments

- `use_gazebo` (default: `true`): Whether to launch Gazebo simulation.
- `rviz` (default: `true`): Whether to launch RViz visualization.

## Customization

- **Robot Name**: Change the `robot_name` variable in [`tow_tractor_bringup.launch.py`](src/tow_tractor_bringup/launch/tow_tractor_bringup.launch.py) to select a different robot variant.
- **Control Mode**: Set `robot_control` to `"diff_drive"` or `"rear_steer"` as needed.

## Adding New Robots

1. Add your URDF/Xacro and meshes to [`src/tow_tractor_description/urdf`](src/tow_tractor_description/urdf).
2. Add a YAML config for your robot in [`src/tow_tractor_description/config`](src/tow_tractor_description/config).
3. Update `robot_name` in the launch file.

## Linting & Tests

Run code style checks with:

```sh
colcon test
```

## License

This project is licensed under the Apache License 2.0 - see the LICENSE files for details.

---

**Maintainer:** Mahmoud Mostafa  
**Contact:** mah2002moud@gmail.com