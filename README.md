# My Robot

This is a ROS 2 Humble package for controlling a two-wheeled mobile robot designed for navigation using NAV2. The project includes both simulation in Gazebo and deployment on a real robot equipped with a Raspberry Pi 4 running Ubuntu and an Arduino Uno for low-level motor control. The robot is also fitted with an RPLidar A1 for 2D mapping and an ESP32 camera.

## Key Features

- **Low-level control**: Implemented on an Arduino Uno, communicating with the Raspberry Pi via the `DiffDriveArduinoHardware` interface.
- **Sensors**: Equipped with an RPLidar A1 (2D LIDAR) and an ESP32 camera for vision.
- **Navigation modes**:
  - **Mapping Mode**: For SLAM using the `slam_toolbox`.
  - **Localization Mode**: For autonomous navigation using NAV2.

## Simulation and Real Robot Support

This package supports both Gazebo simulation and deployment on the real robot. Below are the launch commands to start the robot in different modes:

### Launching Simulation

- **Mapping Mode** (SLAM):

  ```bash
  ros2 launch my_robot full_sim_mapping.launch.py
  ```

- **Localization Mode** (Navigation with Nav2):

  ```bash
  ros2 launch my_robot full_sim_localization.launch.py
  ```

### Launching on the Real Robot

- **Mapping Mode** (SLAM):

  ```bash
  ros2 launch my_robot full_real_robot_mapping.launch.py
  ```

- **Localization Mode** (Navigation with Nav2):

  ```bash
  ros2 launch my_robot full_real_robot_localization.launch.py
  ```

## Package Structure

```text
my_robot/
├── description/                # Robot description files
│   ├── *.xacro                 # Xacro files for robot description
│   └── *.urdf                  # URDF files for the robot
├── launch/                     # Launch files for both simulation and real robot
│   └── *.launch.py             # Launch scripts
├── map/                        # Map files
│   ├── *.yaml                  # YAML map files for localization
│   └── *.pgm                   # PGM files for occupancy grids
├── params/                     # Parameter configuration files
│   └── *.yaml                  # YAML configuration for different nodes
├── rviz/                       # RViz configuration files
│   └── *.rviz                  # RViz setup files
├── worlds/                     # Gazebo world files
│   └── *.world                 # World description for simulation
├── CMakeLists.txt              # CMake configuration
├── LICENSE.md                  # License information (Apache 2.0)
├── package.xml                 # ROS 2 package manifest
└── README.md                   # This README file
```

## Hardware Setup

- **Raspberry Pi 4**: Running Ubuntu and managing high-level navigation tasks.
- **Arduino Uno**: Handling low-level motor control via `DiffDriveArduinoHardware` interface, communicated over a serial connection.
- **Sensors**:
  - **RPLidar A1**: For 2D SLAM and obstacle detection.
  - **ESP32 Camera**: Providing video feedback (optional use).

## Prerequisites

- ROS 2 Humble installed on your machine.
- `slam_toolbox` for SLAM and `nav2` for autonomous navigation.
- Hardware setup according to the specifications.

## Installation

Clone the repository and build it using colcon:

```bash
git clone https://github.com/yourusername/my_robot.git
cd my_robot
colcon build
```

## Usage

Make sure your Arduino Uno is flashed with the correct firmware from [my_robot_arduino_ros](https://github.com/Axelado/my_robot_arduino_ros) and connected to the Raspberry Pi via serial.

### For Simulation

Use the launch commands specified above for either mapping or localization in Gazebo.

### For Real Robot

Launch the respective `full_real_robot_*` launch files for mapping or localization on the physical robot.

## License

This project is licensed under the Apache License 2.0. See the `LICENSE.md` file for details.
