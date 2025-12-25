# ROS 2 Interface Contracts: Physical AI & Humanoid Robotics

**Feature**: 001-humanoid-robotics
**Date**: 2025-12-16
**Status**: Planning Phase

## Overview

This document defines the contractual interfaces for ROS 2 communication in the Physical AI & Humanoid Robotics system. These contracts specify the message types, service interfaces, and topic/service names that ensure proper communication between system components.

## Module 1: Robotic Nervous System Contracts

### Topic Interfaces

#### Joint State Topic
```
Topic: /joint_states
Type: sensor_msgs/JointState
Frequency: 50Hz minimum
QoS: Reliability: Reliable, Durability: Volatile
Description: Publishes current joint positions, velocities, and efforts for all robot joints
```

#### Robot Command Topic
```
Topic: /robot_command
Type: std_msgs/String
Frequency: On-demand
QoS: Reliability: Reliable, Durability: Volatile
Description: Accepts high-level robot commands (STAND_UP, SIT_DOWN, HOME_POSITION, etc.)
```

#### High-Level Command Topic
```
Topic: /high_level_command
Type: std_msgs/String
Frequency: On-demand
QoS: Reliability: Reliable, Durability: Volatile
Description: Accepts natural language or structured commands for robot execution
```

#### Joint Command Topic
```
Topic: /joint_commands
Type: std_msgs/Float64MultiArray
Frequency: 100Hz maximum
QoS: Reliability: Reliable, Durability: Volatile
Description: Direct joint position commands for low-level control
```

#### Robot Status Topic
```
Topic: /robot_status
Type: std_msgs/String
Frequency: 1Hz
QoS: Reliability: Best Effort, Durability: Volatile
Description: Publishes current robot operational status
```

#### Command Feedback Topic
```
Topic: /command_feedback
Type: std_msgs/String
Frequency: On-demand
QoS: Reliability: Best Effort, Durability: Volatile
Description: Provides feedback on command execution status
```

#### Velocity Command Topic
```
Topic: /cmd_vel
Type: geometry_msgs/Twist
Frequency: 10Hz maximum
QoS: Reliability: Reliable, Durability: Volatile
Description: Accepts velocity commands for base movement
```

### Service Interfaces

#### Joint Control Service
```
Service: /control_joint
Type: JointControl (custom)
Request: {joint_name: string, target_position: float64, duration: duration}
Response: {success: bool, error_message: string, actual_position: float64}
Timeout: 5 seconds
Description: Move a specific joint to target position
```

#### Robot State Service
```
Service: /get_robot_state
Type: RobotState (custom)
Request: {include_sensors: bool, include_joints: bool, include_imu: bool}
Response: {joint_states: JointState, imu_data: Imu, sensor_data: [], timestamp: time}
Timeout: 2 seconds
Description: Get current robot state information
```

### Action Interfaces

#### Joint Trajectory Action
```
Action: /joint_trajectory
Type: control_msgs/FollowJointTrajectory
Goal: control_msgs/FollowJointTrajectoryGoal
Feedback: control_msgs/FollowJointTrajectoryFeedback
Result: control_msgs/FollowJointTrajectoryResult
Timeout: 30 seconds
Description: Execute joint trajectory with feedback
```

#### Navigation Action
```
Action: /navigate_to_pose
Type: nav2_msgs/NavigateToPose
Goal: geometry_msgs/PoseStamped
Feedback: nav2_msgs/NavigateToPoseFeedback
Result: nav2_msgs/NavigateToPoseResult
Timeout: 300 seconds
Description: Navigate robot to specified pose
```

## Module 2: Digital Twin Contracts

### Simulation Topics

#### Simulation Clock
```
Topic: /clock
Type: rosgraph_msgs/Clock
Frequency: Simulation rate
QoS: Reliability: Reliable, Durability: Transient Local
Description: Provides simulation time for all nodes
```

#### Model State
```
Topic: /model_states
Type: gazebo_msgs/ModelStates
Frequency: 30Hz
QoS: Reliability: Best Effort, Durability: Volatile
Description: Publishes states of all models in simulation
```

#### Camera Image Topics
```
Topic: /camera/rgb/image_raw
Type: sensor_msgs/Image
Frequency: 30Hz
QoS: Reliability: Best Effort, Durability: Volatile
Description: Raw RGB camera image from simulation

Topic: /camera/depth/image_raw
Type: sensor_msgs/Image
Frequency: 30Hz
QoS: Reliability: Best Effort, Durability: Volatile
Description: Raw depth image from simulation camera
```

#### LiDAR Data
```
Topic: /laser_scan
Type: sensor_msgs/LaserScan
Frequency: 10Hz
QoS: Reliability: Best Effort, Durability: Volatile
Description: 2D LiDAR scan data from simulation
```

#### IMU Data
```
Topic: /imu/data
Type: sensor_msgs/Imu
Frequency: 100Hz
QoS: Reliability: Best Effort, Durability: Volatile
Description: IMU sensor data from simulation
```

## Module 3: AI-Robot Brain Contracts

### Perception Topics

#### Object Detection
```
Topic: /perception/detection
Type: vision_msgs/Detection2DArray
Frequency: 5Hz
QoS: Reliability: Best Effort, Durability: Volatile
Description: Object detection results from perception pipeline
```

#### Semantic Segmentation
```
Topic: /perception/segmentation
Type: sensor_msgs/Image
Frequency: 5Hz
QoS: Reliability: Best Effort, Durability: Volatile
Description: Semantic segmentation results
```

#### SLAM Map
```
Topic: /map
Type: nav_msgs/OccupancyGrid
Frequency: On-change
QoS: Reliability: Reliable, Durability: Transient Local
Description: Occupancy grid map from SLAM system
```

#### Robot Pose
```
Topic: /amcl_pose
Type: geometry_msgs/PoseWithCovarianceStamped
Frequency: 1Hz
QoS: Reliability: Best Effort, Durability: Volatile
Description: Robot pose estimate from localization
```

### Navigation Topics

#### Global Path
```
Topic: /plan
Type: nav_msgs/Path
Frequency: On-request
QoS: Reliability: Best Effort, Durability: Volatile
Description: Global path plan from navigation system
```

#### Local Path
```
Topic: /local_plan
Type: nav_msgs/Path
Frequency: 10Hz
QoS: Reliability: Best Effort, Durability: Volatile
Description: Local path plan for obstacle avoidance
```

## Module 4: Vision-Language-Action Contracts

### Voice Processing Topics

#### Speech Recognition
```
Topic: /speech_recognition
Type: SpeechRecognitionResult (custom)
Frequency: On-voice-activity
QoS: Reliability: Best Effort, Durability: Volatile
Description: Results from speech recognition system
```

#### Natural Language Command
```
Topic: /natural_language_command
Type: NaturalLanguageCommand (custom)
Frequency: On-voice-interpretation
QoS: Reliability: Reliable, Durability: Volatile
Description: Interpreted natural language commands
```

### Planning Topics

#### Task Plan
```
Topic: /task_plan
Type: TaskPlan (custom)
Frequency: On-plan-generation
QoS: Reliability: Reliable, Durability: Volatile
Description: Generated task plans from LLM planner
```

#### Action Execution
```
Topic: /action_execution
Type: RobotAction (custom)
Frequency: On-action-start
QoS: Reliability: Reliable, Durability: Volatile
Description: Action execution commands
```

## Quality of Service (QoS) Guidelines

### Reliability Settings
- **Reliable**: For critical control and state information
- **Best Effort**: For sensor data and visualization where occasional packet loss is acceptable

### Durability Settings
- **Volatile**: For real-time streaming data
- **Transient Local**: For static information like maps that should be available to late-joining subscribers

## Error Handling Contracts

### Timeout Specifications
- Control services: 5 seconds
- State queries: 2 seconds
- Navigation actions: 300 seconds
- Perception processing: 10 seconds

### Fallback Behaviors
- If joint control fails, robot enters safe state
- If perception fails, robot uses conservative navigation
- If voice recognition fails, robot waits for alternative input
- If navigation fails, robot stops and reports error

## Validation Criteria

### Interface Compliance
- All messages must follow ROS 2 message definitions
- Topics must maintain specified frequencies
- Services must respond within timeout limits
- Actions must provide appropriate feedback

### Performance Requirements
- Control loops: <10ms latency
- Perception processing: <100ms processing time
- Navigation planning: <1s for simple plans
- Voice processing: <500ms from input to action

### Safety Requirements
- All interfaces must include safety checks
- Emergency stop capability through all control paths
- Error states must be clearly communicated
- Graceful degradation when components fail