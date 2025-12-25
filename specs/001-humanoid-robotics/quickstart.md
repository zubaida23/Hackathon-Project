# Quickstart Guide: Physical AI & Humanoid Robotics

**Feature**: 001-humanoid-robotics
**Date**: 2025-12-16
**Status**: Planning Phase

## Overview

This quickstart guide provides the essential steps to set up and run the Physical AI & Humanoid Robotics system. It covers the minimum viable setup to demonstrate the core concepts across all four modules.

## Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04 LTS (recommended) or compatible Linux distribution
- **RAM**: 8GB minimum, 16GB recommended
- **CPU**: Multi-core processor (Intel i5 or equivalent)
- **GPU**: NVIDIA GPU with CUDA support recommended (for Isaac acceleration)
- **Storage**: 20GB free space minimum

### Software Dependencies
- ROS 2 Humble Hawksbill
- Gazebo Garden or Fortress
- Python 3.8+
- Git
- CMake 3.8+
- NVIDIA Isaac Sim (optional for acceleration)

## Installation Steps

### 1. Install ROS 2 Humble Hawksbill

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-rosdep2
sudo apt install -y python3-colcon-common-extensions

# Source ROS 2
source /opt/ros/humble/setup.bash
```

### 2. Install Gazebo

```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo.list > /dev/null

sudo apt update
sudo apt install -y gz-harmonic
```

### 3. Set up Workspace

```bash
# Create workspace
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build workspace
colcon build --packages-select ros2_module
source install/setup.bash
```

### 4. Install Additional Dependencies

```bash
# Python dependencies
pip3 install numpy scipy matplotlib pyyaml

# For voice processing (if using Whisper)
pip3 install openai-whisper

# For documentation
npm install -g docusaurus
```

## Module 1: Robotic Nervous System

### Launch the Basic ROS 2 System

```bash
# Terminal 1: Launch sensor publisher
source ~/humanoid_ws/install/setup.bash
ros2 run ros2_module sensor_publisher

# Terminal 2: Launch robot controller
source ~/humanoid_ws/install/setup.bash
ros2 run ros2_module robot_controller

# Terminal 3: Send commands
source ~/humanoid_ws/install/setup.bash
ros2 topic pub /robot_command std_msgs/String "data: 'STAND_UP'"
```

### Verify URDF Model

```bash
# Check if URDF loads correctly
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(xacro ~/humanoid_ws/src/ros2_module/urdf/humanoid_robot.xacro)

# Visualize in RViz
rviz2
```

## Module 2: Digital Twin

### Launch Gazebo Simulation

```bash
# Terminal 1: Launch Gazebo with humanoid model
source ~/humanoid_ws/install/setup.bash
gz sim -r -v 1 ~/humanoid_ws/src/ros2_module/simulation/worlds/humanoid_world.sdf

# Terminal 2: Launch ROS 2 bridge
source ~/humanoid_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge
```

### Verify Simulation

```bash
# Check if simulation topics are available
ros2 topic list | grep /world/

# Verify sensor data
ros2 topic echo /camera/rgb/image_raw
ros2 topic echo /imu/data
```

## Module 3: AI-Robot Brain

### Launch Perception System

```bash
# Terminal 1: Launch perception pipeline
source ~/humanoid_ws/install/setup.bash
ros2 launch perception_pipeline.launch.py

# Terminal 2: Launch navigation system
source ~/humanoid_ws/install/setup.bash
ros2 launch nav2_bringup navigation_launch.py
```

### Test SLAM

```bash
# Terminal 1: Launch SLAM
source ~/humanoid_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py

# Terminal 2: Move the robot and observe map building
source ~/humanoid_ws/install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5}; angular: {z: 0.2}"
```

## Module 4: Vision-Language-Action

### Launch Voice Processing

```bash
# Terminal 1: Launch voice recognition
source ~/humanoid_ws/install/setup.bash
ros2 run vla_system voice_processor

# Terminal 2: Launch command interpreter
source ~/humanoid_ws/install/setup.bash
ros2 run ros2_module command_interpreter

# Terminal 3: Test voice command
# Speak into microphone: "Stand up"
```

## Integrated System Test

### Launch Complete System

```bash
# Terminal 1: Launch all core systems
source ~/humanoid_ws/install/setup.bash
ros2 launch humanoid_robot_system.launch.py

# Terminal 2: Test integrated functionality
source ~/humanoid_ws/install/setup.bash
ros2 topic pub /high_level_command std_msgs/String "data: 'Walk forward and avoid obstacles'"
```

## Documentation Setup

### Build and Serve Documentation

```bash
# Navigate to docs directory
cd ~/humanoid_ws/docs

# Install dependencies
npm install

# Build documentation
npm run build

# Serve locally for development
npm run serve
```

## Troubleshooting

### Common Issues

#### ROS 2 Not Found
- Ensure ROS 2 is properly sourced: `source /opt/ros/humble/setup.bash`
- Check installation: `echo $ROS_DISTRO`

#### Gazebo Not Launching
- Verify installation: `gz --version`
- Check for missing dependencies: `sudo apt install gz-harmonic`

#### Python Import Errors
- Ensure correct Python version: `python3 --version`
- Reinstall dependencies: `pip3 install -r requirements.txt`

#### URDF Loading Issues
- Verify xacro installation: `which xacro`
- Check file permissions: `ls -la urdf/humanoid_robot.xacro`

### Verification Commands

```bash
# Check ROS 2 environment
printenv | grep ROS

# List available packages
ros2 pkg list | grep humanoid

# Check active topics
ros2 topic list

# Verify nodes are running
ros2 node list
```

## Next Steps

1. **Module 1 Deep Dive**: Explore ROS 2 concepts and message types in detail
2. **Module 2 Advanced Simulation**: Configure physics parameters and sensor models
3. **Module 3 AI Integration**: Implement perception and navigation algorithms
4. **Module 4 VLA System**: Develop voice processing and LLM integration
5. **Capstone Project**: Integrate all modules into a complete autonomous system

## Resources

- ROS 2 Documentation: https://docs.ros.org/en/humble/
- Gazebo Documentation: https://gazebosim.org/docs/
- Docusaurus Guide: https://docusaurus.io/docs
- Robotics Stack Exchange: https://robotics.stackexchange.com/