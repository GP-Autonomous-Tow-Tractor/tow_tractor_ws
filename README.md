# Tow Tractor ROS2 Simulation & Control

This repository contains a modular ROS2 workspace for simulating, describing, and controlling a tow tractor robot. It supports both hardware and simulation (Gazebo) modes, and is organized into several packages for description, control, hardware interface, and simulation integration.

## Workspace Structure

```
.
├── src/
│   ├── tow_tractor/                # Custom message definitions (e.g., ModelInfo.msg)
│   ├── tow_tractor_bringup/        # Launch files and overall system bringup
│   ├── tow_tractor_control/        # Controllers (diff drive, rear steer)
│   ├── tow_tractor_description/    # Robot description, URDF, meshes, model info publisher
│   └── tow_tractor_hardware/       # Hardware interface nodes (sensors, actuators)
│   └── tow_tractor_gazebo/         # Gazebo simulation resources (worlds, models)
└── README.md
```

## Main Features

- **Robot Description**: URDF/Xacro files, meshes, and configuration in [`src/tow_tractor_description`](src/tow_tractor_description).
- **Model Info Publisher**: Publishes robot parameters as a ROS2 topic ([`model_info_publisher_node.py`](src/tow_tractor_description/tow_tractor_description/model_info_publisher_node.py)).
- **Controllers**: Diff drive and rear wheel steering controllers ([`diff_drive_controller_node.py`](src/tow_tractor_control/tow_tractor_control/diff_drive_controller_node.py)).
- **Hardware Interface**: Nodes for sensor reading and actuator command ([`sensor_receiver_node`](src/tow_tractor_hardware/tow_tractor_hardware/sensor_receiver_node.py), [`actuators_sender_node`](src/tow_tractor_hardware/tow_tractor_hardware/actuators_sender_node.py)).
- **Gazebo Integration**: Launches Gazebo simulation with robot and world ([`tow_tractor_bringup.launch.py`](src/tow_tractor_bringup/launch/tow_tractor_bringup.launch.py)).
- **SLAM & Navigation**: Integrates with SLAM Toolbox and navigation stack.
- **RViz Visualization**: Pre-configured RViz setups for visualization.

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

- To run in hardware mode, set `use_gazebo:=false`:

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

See individual package `LICENSE` or `package.xml` files.

---

**Maintainer:** Mahmoud  
**Contact:** mah2002moud@gmail.com