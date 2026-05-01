# Admittance Controller for UR5 with FT-300 Force Sensor

A ROS-based admittance control implementation for Universal Robots UR5/UR10 arms equipped with a Robotiq FT-300 force/torque sensor. This controller enables compliant human-robot interaction by generating desired end-effector motion (twist) from external wrench measurements and control inputs.

Partly based on [robot_admittance_controller](https://github.com/nbfigueroa/robot_admittance_controller) by Nadia Figueroa.

---

## Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Admittance Control Theory](#admittance-control-theory)
- [Configuration](#configuration)
- [Safety Features](#safety-features)
- [License](#license)

---

## Overview

This package implements an **admittance control law** for velocity/position-controlled robotic arms that are equipped with an external force/torque sensor. Unlike impedance control which requires torque-controlled robots, admittance control interprets force measurements as desired motion, making it suitable for standard industrial robots like the UR series.

### Key Capabilities

- Real-time admittance control loop running at **150 Hz**
- 6-DOF end-effector compliance (linear + angular)
- Configurable mass and damping matrices
- Workspace boundary protection with repulsive velocity field
- Velocity and acceleration saturation for safety
- Support for both **simulated** and **real** robot configurations
- TF-based frame transformations for sensor data alignment

### Tested Platforms

- **UR10** velocity-controlled robot with Robotiq FT-300 sensor
- **UR5** with FT-300 force/torque sensor

---

## Project Structure

```
admittance-controller_UR5/
├── robot_admittance_controller/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── README.md
│   │
│   ├── config/                           # Admittance parameter configs
│   │   ├── admittance_params.yaml        # Default simulation parameters
│   │   ├── arm_admittance_params_real.yaml   # Real robot parameters (UR5)
│   │   ├── ur10_admittance_params.yaml       # UR10 simulation parameters
│   │   └── ur10_admittance_params_real.yaml  # UR10 real robot parameters
│   │
│   ├── include/
│   │   └── AdmittanceController.h        # Main controller class header
│   │
│   ├── launch/                           # ROS launch files
│   │   ├── arm_admittance_controller.launch          # Generic arm launch
│   │   ├── ur10_admittance_controller.launch         # UR10 simulation
│   │   ├── ur10_admittance_controller_real.launch    # UR10 real robot
│   │   ├── ur5_ft_upload.launch                      # Load UR5+FT300 model
│   │   └── view_ur5_ft.launch                        # RViz visualization
│   │
│   ├── src/                              # Source files
│   │   ├── AdmittanceController.cpp      # Core admittance control logic
│   │   ├── admittance_controller_node.cpp    # ROS node main entry
│   │   └── readForce.cpp                 # FT sensor reader / zeroing tool
│   │
│   ├── Scripts/
│   │   └── readForceSensor.py            # Python script for sensor reading
│   │
│   └── ur_description/                   # UR5 URDF & mesh files
│       ├── urdf/
│       │   ├── ur5.urdf.xacro            # UR5 robot macro
│       │   ├── ur5_ft_robot.urdf.xacro   # UR5 + FT-300 combined model
│       │   ├── ur5_robot.urdf.xacro      # UR5 standalone model
│       │   ├── robotiq_ft300.urdf.xacro  # FT-300 sensor macro
│       │   └── common.gazebo.xacro       # Gazebo simulation definitions
│       ├── meshes/ur5/                   # STL/DAE collision & visual meshes
│       ├── launch/                       # Model upload & RViz launch files
│       └── cfg/
│           └── view_robot.rviz           # RViz configuration
│
└── README.md
```

---

## Prerequisites

- **ROS** (tested on Indigo/Kinetic, compatible with Melodic/Noetic with minor adjustments)
- **catkin** build tools
- **Eigen3** linear algebra library
- **`robotiq_ft_sensor`** ROS package (for FT-300 sensor driver)
- **`tf`** and **`tf_conversions`** ROS packages
- **Universal Robots ROS driver** (`ur_modern_driver` or `ur_robot_driver`)

---

## Installation

1. Clone this repository into your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/admittance-controller_UR5.git
```

2. Install dependencies (ensure `robotiq_ft_sensor` is available in your workspace).

3. Build the package:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## Usage

### Simulation (UR10)

```bash
roslaunch arm_admittance_controller ur10_admittance_controller.launch
```

### Real Robot (UR5 + FT-300)

1. Launch the robot driver and FT sensor:

```bash
# Start UR5 driver (example with ur_modern_driver)
roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.1.10

# Start FT-300 sensor node
roslaunch robotiq_ft_sensor rq_sensor.launch
```

2. Load the UR5+FT300 model:

```bash
roslaunch arm_admittance_controller ur5_ft_upload.launch
```

3. Launch the admittance controller:

```bash
roslaunch arm_admittance_controller arm_admittance_controller.launch
```

### Visualize in RViz

```bash
roslaunch arm_admittance_controller view_ur5_ft.launch
```

### Zero the FT Sensor

The `readForce` node can be used to monitor sensor data and periodically zero the sensor:

```bash
rosrun arm_admittance_controller readForce
```

---

## Admittance Control Theory

The core control law implemented is a **mass-damper system**:

```
M_a * a_des = -D_a * v_des + F_ext + F_ctrl
```

Where:

| Symbol | Description |
|--------|-------------|
| `M_a` | Desired mass matrix (6x6 diagonal) |
| `D_a` | Desired damping matrix (6x6 diagonal) |
| `F_ext` | External wrench from FT sensor (force/torque) |
| `F_ctrl` | Control wrench from higher-level controller |
| `v_des` | Desired end-effector twist (velocity) |

The acceleration is integrated to obtain the desired velocity command sent to the robot's velocity controller.

---

## Configuration

Admittance behavior is configured via YAML files in `config/`.

### Example Parameters (`admittance_params.yaml`)

```yaml
# Mass matrix (translational + rotational)
mass_arm: [2,0,0,0,0,0,
           0,2,0,0,0,0,
           0,0,2,0,0,0,
           0,0,0,1,0,0,
           0,0,0,0,1,0,
           0,0,0,0,0,1]

# Damping matrix
damping_arm: [12,0,0,0,0,0,
              0,12,0,0,0,0,
              0,0,12,0,0,0,
              0,0,0,7,0,0,
              0,0,0,0,7,0,
              0,0,0,0,0,7]

# Workspace limits [x_min, x_max, y_min, y_max, z_min, z_max] (meters)
workspace_limits: [0.15, 1.75, -1.75, 1.75, 0.30, 1.4]

# Safety limits
arm_max_vel: 0.5    # Maximum linear velocity (m/s)
arm_max_acc: 1.0    # Maximum linear acceleration (m/s^2)
```

### Tuning Guidelines

- **Lower mass / damping**: More compliant, easier to move by hand
- **Higher mass / damping**: Stiffer behavior, better trajectory tracking
- **Real robots**: Typically require higher damping (e.g., 60) than simulation (e.g., 12) due to unmodeled dynamics

---

## Safety Features

The controller includes multiple safety mechanisms:

1. **Velocity Saturation**: Limits end-effector linear velocity to `arm_max_vel`
2. **Acceleration Saturation**: Prevents dangerously high accelerations
3. **Workspace Limits**: Defines a bounded operational region; if the robot exits this region, a repulsive velocity field drives it back
4. **Angular Velocity Limits**: Hard-capped at 0.3 rad/s for rotational components
5. **Near-zero Filtering**: Very small velocities are clamped to zero to prevent jitter
6. **TF Validation**: Controller waits for all required transforms before enabling control

---

## ROS Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `topic_arm_pose` | `geometry_msgs/Pose` | End-effector pose in world frame |
| `topic_arm_twist` | `geometry_msgs/Twist` | End-effector twist in world frame |
| `topic_external_wrench` | `geometry_msgs/WrenchStamped` | FT sensor wrench |
| `topic_control_wrench` | `geometry_msgs/WrenchStamped` | External control wrench input |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `topic_arm_command` | `geometry_msgs/Twist` | Desired twist command to robot driver |

---

## License

This project is licensed under the **MIT License**. See the `LICENSE` file for details.

The original `robot_admittance_controller` code by Nadia Figueroa is also MIT licensed. The UR description package (`ur_description`) is licensed under BSD.
