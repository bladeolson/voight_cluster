# TE Arm - Robot Arm Control (手)

**VOIGHT CLUSTER** - 6-DOF Robot Arm with ROS2 Integration

## Overview

This package provides ROS2 control for a 6-servo robot arm connected via Arduino to the TE node (`te.local`).

```
┌─────────────────┐     Serial      ┌─────────────┐     ROS2      ┌─────────────┐
│  Arduino Mega   │◄───────────────►│  te.local   │◄─────────────►│  ROS2 Nodes │
│  (6 Servos)     │   /dev/ttyUSB0  │  (Bridge)   │   Topics/Srv  │  (Control)  │
└─────────────────┘                 └─────────────┘               └─────────────┘
```

## Hardware Setup

### Wiring (Arduino)

| Servo | Arduino Pin | Joint |
|-------|-------------|-------|
| J1 | D3 | Base rotation |
| J2 | D5 | Shoulder |
| J3 | D6 | Elbow |
| J4 | D9 | Wrist pitch |
| J5 | D10 | Wrist roll |
| J6 | D11 | Gripper |

### Power
- **DO NOT power servos from Arduino!** Use external 5-6V power supply
- Connect servo ground to Arduino ground
- Recommended: 5V 3A+ power supply for 6 servos

## Installation

### 1. Flash Arduino Firmware

```bash
# Install Arduino CLI (if not installed)
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Compile and upload
cd arduino/te_arm_firmware
arduino-cli compile --fqbn arduino:avr:mega .
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:mega .
```

### 2. Install ROS2 (on te.local or control machine)

```bash
# For Ubuntu 22.04 - ROS2 Humble
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop ros-humble-robot-state-publisher
```

### 3. Install Python Dependencies

```bash
pip install pyserial
```

### 4. Build ROS2 Package

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select te_arm
source install/setup.bash
```

## Usage

### Quick Test (No ROS)

```bash
# Test the serial bridge directly
python3 te_arm_bridge.py
```

### ROS2 Launch

```bash
# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

# Launch arm controller
ros2 launch te_arm te_arm.launch.py

# With RViz visualization
ros2 launch te_arm te_arm_rviz.launch.py
```

### Teleop Control

```bash
# In another terminal
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 run te_arm arm_teleop.py
```

### ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/te_arm/joint_states` | `sensor_msgs/JointState` | Current joint positions |
| `/te_arm/joint_commands` | `sensor_msgs/JointState` | Target joint positions |
| `/te_arm/gripper_command` | `std_msgs/Float64` | Gripper position (0-180) |
| `/te_arm/status` | `std_msgs/String` | Status JSON |

### ROS2 Services

```bash
# Move to home position
ros2 service call /te_arm/home std_srvs/srv/Trigger

# Enable servos
ros2 service call /te_arm/enable std_srvs/srv/Trigger

# Disable servos (release)
ros2 service call /te_arm/disable std_srvs/srv/Trigger
```

### Command Line Control

```bash
# Move joints (radians: 1.57 = 90°)
ros2 topic pub --once /te_arm/joint_commands sensor_msgs/msg/JointState \
  "{position: [1.57, 1.57, 1.57, 1.57, 1.57, 1.57]}"

# Set gripper
ros2 topic pub --once /te_arm/gripper_command std_msgs/msg/Float64 "{data: 90.0}"
```

## Arduino Protocol

The Arduino accepts JSON commands over serial at 115200 baud:

```json
// Move to position
{"cmd":"move","joints":[90,90,90,90,90,90],"smooth":true}

// Go home
{"cmd":"home"}

// Get status
{"cmd":"status"}

// Control gripper
{"cmd":"grip","value":45}

// Enable/disable servos
{"cmd":"enable"}
{"cmd":"disable"}
```

Response format:
```json
{"ok":true,"joints":[90,90,90,90,90,90],"enabled":true}
```

## Customization

### Joint Limits

Edit `arduino/te_arm_firmware/te_arm_firmware.ino`:

```cpp
const int JOINT_MIN[NUM_JOINTS] = {0, 0, 0, 0, 0, 0};
const int JOINT_MAX[NUM_JOINTS] = {180, 180, 180, 180, 180, 180};
```

### Servo Pins

```cpp
const int SERVO_PINS[NUM_JOINTS] = {3, 5, 6, 9, 10, 11};
```

### URDF Model

Edit `ros2_ws/src/te_arm/urdf/te_arm.urdf` to match your arm's dimensions.

## Troubleshooting

### Arduino not detected
```bash
# Check USB devices
ls -la /dev/ttyUSB* /dev/ttyACM*

# Check permissions
sudo usermod -a -G dialout $USER
# (logout/login required)
```

### Servos jittering
- Check power supply capacity (need 3A+ for 6 servos)
- Add capacitor (100-1000μF) across power rails
- Reduce movement speed in firmware

### ROS2 connection issues
```bash
# Check if nodes are running
ros2 node list
ros2 topic list

# Monitor arm status
ros2 topic echo /te_arm/status
```

## Integration with VOIGHT CLUSTER

This arm is part of the TE node (手 - hand/limbs) in the VOIGHT CLUSTER.
The FastAPI server at `te.local:8027` can expose arm control via HTTP.

See `zen_te_server.py` for HTTP API integration.

