# Keyboard-control-Robot

# Keyboard Control - ROS2 Differential Drive Controller

A ROS2 package implementing a differential drive base controller with keyboard teleoperation capabilities. This package provides nodes for serial communication with the motor controllers, base control, and keyboard teleoperation.

## Features

- Differential drive base controller with forward kinematics and odometry
- Real-time keyboard teleoperation
- Serial communication interface for motor control
- Configurable parameters for wheel geometry and control
- Heartbeat monitoring for system health
- Command velocity timeout safety feature
- Multi-threaded execution for improved performance

## Prerequisites

- ROS2 (tested on Humble)
- Python 3.8+
- libserial-dev
- pynput (for keyboard control) [This library is not to be used using SSH]

## Installation

1. Create a ROS2 workspace (if you haven't already):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone this repository:
```bash
git clone https://github.com/Shashanksharma280201/Keyboard-control-Robot.git 
```

3. Install dependencies:
```bash
sudo apt-get install libserial-dev
pip3 install pynput
```

4. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select flo_base
source install/setup.bash
```

## Usage

### Launch the Base Controller

The package provides a launch file that starts all necessary nodes:

```bash
ros2 launch flo_base flo_base.launch.py
```

This will start:
- Serial port node for motor communication
- Base controller node
- Keyboard controller node

### Keyboard Control

The keyboard controller supports the following commands:

- **Movement Controls:**
  - Arrow Keys or WASD:
    - ↑ or W: Move forward
    - ↓ or S: Move backward
    - ← or A: Rotate left
    - → or D: Rotate right

- **Speed Control:**
  - +: Increase speed
  - -: Decrease speed
  - Q: Quit

### Configuration

Parameters can be configured in the `config/base_params.yaml` file:

#### Base Controller Parameters
- `wheel_diameter`: Diameter of the wheels (default: 0.165 m)
- `wheel_to_wheel_separation`: Distance between wheels (default: 0.37 m)
- `wheel_separation_multiplier`: Calibration factor for wheel separation (default: 1.0)
- `cmd_vel_timeout`: Timeout for command velocity messages (default: 0.5 s)
- `odom_hz`: Odometry update frequency (default: 30 Hz)

#### Keyboard Controller Parameters
- `linear_speed`: Default linear speed (default: 0.36 m/s)
- `angular_speed`: Default angular speed (default: 0.5 rad/s)
- `update_rate`: Control update rate (default: 0.1 s)

## Nodes

### 1. Base Controller (`flo_base_diff`)
- Implements differential drive kinematics
- Publishes odometry data
- Handles motor control commands
- Monitors system health through heartbeat

### 2. Keyboard Controller
- Provides real-time keyboard teleoperation
- Adjustable speed controls
- Emergency stop functionality

### 3. Serial Port Node
- Handles communication with motor controllers
- Processes joint state feedback
- Implements reliable serial communication

## Topics

### Published Topics
- `/cmd_vel` (geometry_msgs/Twist): Command velocity
- `/odom/wheel_encoder` (nav_msgs/Odometry): Wheel odometry
- `/motion/status` (std_msgs/Bool): Robot motion status

### Subscribed Topics
- `/serial_port_drive/out` (std_msgs/String): Serial port output
- `/cmd_vel` (geometry_msgs/Twist): Command velocity input


- Your Name - [shashank.sharma.280201@gmail.com]

## Acknowledgments

- ROS2 Community
- Contributors and testers
