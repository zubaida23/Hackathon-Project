# Data Model: Physical AI & Humanoid Robotics

**Feature**: 001-humanoid-robotics
**Date**: 2025-12-16
**Status**: Planning Phase

## Overview

This document defines the key data structures and message types used across the Physical AI & Humanoid Robotics system. The data model encompasses ROS 2 message types, simulation data, perception outputs, and command structures used in the four modules.

## Module 1: The Robotic Nervous System (ROS 2)

### Core Message Types

#### Joint State Messages
```
sensor_msgs/JointState
- header: std_msgs/Header
- name: string[] (joint names)
- position: float64[] (joint positions in radians)
- velocity: float64[] (joint velocities in rad/s)
- effort: float64[] (joint efforts in Nm)
```

#### Robot Command Messages
```
Custom: RobotCommand
- header: std_msgs/Header
- command_type: string (e.g., "STAND_UP", "SIT_DOWN", "MOVE_JOINT")
- target_joint: string (for joint-specific commands)
- target_position: float64 (for position commands)
- parameters: string[] (additional command parameters)
```

#### Sensor Data Messages
```
sensor_msgs/Imu
- header: std_msgs/Header
- orientation: geometry_msgs/Quaternion
- angular_velocity: geometry_msgs/Vector3
- linear_acceleration: geometry_msgs/Vector3
```

```
sensor_msgs/LaserScan
- header: std_msgs/Header
- angle_min: float32
- angle_max: float32
- angle_increment: float32
- time_increment: float32
- scan_time: float32
- range_min: float32
- range_max: float32
- ranges: float32[]
- intensities: float32[]
```

### Service Interfaces

#### Joint Control Service
```
Request: JointControlRequest
- joint_name: string
- target_position: float64
- duration: duration

Response: JointControlResponse
- success: bool
- error_message: string
- actual_position: float64
```

#### Robot State Service
```
Request: RobotStateRequest
- include_sensors: bool
- include_joints: bool
- include_imu: bool

Response: RobotStateResponse
- joint_states: sensor_msgs/JointState
- imu_data: sensor_msgs/Imu
- sensor_data: sensor_msgs/[]
- timestamp: time
```

## Module 2: The Digital Twin (Gazebo & Unity)

### Simulation State Messages

#### Simulation Environment State
```
Custom: SimulationState
- header: std_msgs/Header
- simulation_time: float64
- real_time_factor: float64
- paused: bool
- models: ModelState[]
```

#### Model State Message
```
Custom: ModelState
- model_name: string
- pose: geometry_msgs/Pose
- twist: geometry_msgs/Twist
- reference_frame: string
```

### Physics Parameters

#### Physics Configuration
```
Custom: PhysicsConfig
- gravity: geometry_msgs/Vector3 (default: [0, 0, -9.81])
- ode_config: ODEConfig
  - ode_solver_type: string
  - ode_min_step_size: float64
  - ode_max_step_size: float64
  - ode_max_contacts: int32
- bullet_config: BulletConfig
  - bullet_solver_iterations: int32
  - bullet_solver_type: string
```

### Sensor Simulation Data

#### Camera Image with Metadata
```
sensor_msgs/Image with CameraInfo
- image: sensor_msgs/Image
- camera_info: sensor_msgs/CameraInfo
  - header: std_msgs/Header
  - height: uint32
  - width: uint32
  - distortion_model: string
  - D: float64[] (distortion coefficients)
  - K: float64[9] (intrinsic matrix)
  - R: float64[9] (rectification matrix)
  - P: float64[12] (projection matrix)
```

#### Depth Camera Data
```
Custom: DepthImage
- header: std_msgs/Header
- rgb_image: sensor_msgs/Image
- depth_image: sensor_msgs/Image
- camera_info: sensor_msgs/CameraInfo
- min_depth: float32
- max_depth: float32
```

## Module 3: The AI-Robot Brain (NVIDIA Isaac)

### Perception Data

#### Object Detection Results
```
vision_msgs/Detection2DArray
- header: std_msgs/Header
- detections: vision_msgs/Detection2D[]
```

```
vision_msgs/Detection2D
- header: std_msgs/Header
- results: vision_msgs/ObjectHypothesisWithPose[]
- bbox: vision_msgs/BoundingBox2D
```

#### SLAM Map and Pose Data
```
nav_msgs/OccupancyGrid (Map)
- header: std_msgs/Header
- info: nav_msgs/MapMetaData
- data: int8[]

geometry_msgs/PoseWithCovarianceStamped (Robot Pose)
- header: std_msgs/Header
- pose: geometry_msgs/PoseWithCovariance
```

### Navigation Data

#### Path Planning
```
nav_msgs/Path
- header: std_msgs/Header
- poses: geometry_msgs/PoseStamped[]
```

#### Goal and Feedback
```
geometry_msgs/PoseStamped
- header: std_msgs/Header
- pose: geometry_msgs/Pose

action_msgs/GoalStatusArray
- status_list: action_msgs/GoalStatus[]
```

### Isaac-Specific Messages

#### Isaac Perception Pipeline Results
```
Custom: IsaacPerceptionResults
- header: std_msgs/Header
- detection_results: vision_msgs/Detection2DArray
- segmentation_results: sensor_msgs/Image
- depth_results: sensor_msgs/Image
- processing_time: float64
- confidence_threshold: float64
```

## Module 4: Vision-Language-Action (VLA)

### Voice Processing Data

#### Speech Recognition Results
```
Custom: SpeechRecognitionResult
- header: std_msgs/Header
- transcript: string
- confidence: float64
- language: string
- audio_duration: float64
- timestamp: time
```

#### Natural Language Command
```
Custom: NaturalLanguageCommand
- header: std_msgs/Header
- raw_command: string
- interpreted_command: string
- command_type: string
- entities: Entity[]
- confidence: float64
```

#### Entity Recognition
```
Custom: Entity
- text: string
- entity_type: string (e.g., "object", "location", "action")
- confidence: float64
- start_index: int32
- end_index: int32
```

### Planning and Action Data

#### Task Plan
```
Custom: TaskPlan
- header: std_msgs/Header
- plan_id: string
- tasks: Task[]
- priority: int32
- estimated_duration: duration
- dependencies: string[]
```

#### Task Definition
```
Custom: Task
- task_id: string
- task_type: string (e.g., "navigation", "manipulation", "perception")
- parameters: string[]
- required_resources: string[]
- success_criteria: string[]
- timeout: duration
```

#### Action Execution
```
Custom: RobotAction
- header: std_msgs/Header
- action_type: string
- action_parameters: string[]
- target_pose: geometry_msgs/Pose (if applicable)
- target_object: string (if applicable)
- execution_priority: int32
```

## Cross-Module Data Structures

### Robot State Aggregation
```
Custom: CompleteRobotState
- header: std_msgs/Header
- joint_states: sensor_msgs/JointState
- sensor_data: SensorData[]
- pose: geometry_msgs/PoseWithCovarianceStamped
- velocity: geometry_msgs/TwistWithCovarianceStamped
- battery_level: float64
- operational_status: string
- active_goals: string[]
```

### Sensor Data Container
```
Custom: SensorData
- sensor_name: string
- sensor_type: string
- data: string (serialized sensor-specific data)
- timestamp: time
- frame_id: string
- status: int32
```

### System Configuration
```
Custom: SystemConfiguration
- header: std_msgs/Header
- robot_model: string
- simulation_active: bool
- perception_enabled: bool
- navigation_enabled: bool
- voice_enabled: bool
- ai_planning_enabled: bool
- configuration_parameters: Parameter[]
```

### Parameter Definition
```
Custom: Parameter
- name: string
- value: string
- parameter_type: string (e.g., "double", "int", "string", "bool")
- description: string
- min_value: string (for numeric types)
- max_value: string (for numeric types)
```

## Data Flow Patterns

### Real-time Control Loop
1. Sensor data published (JointState, IMU, Camera, etc.)
2. Perception processes sensor data
3. Planning module receives perception results
4. Action execution publishes commands
5. Robot executes commands
6. New sensor data closes the loop

### Simulation-Reality Bridge
1. Simulation publishes perfect state information
2. Noise models add realistic variations
3. Processed data matches real sensor characteristics
4. Commands from real robot can be simulated
5. Validation metrics compare simulation vs reality

### AI Integration Pipeline
1. Voice input converted to text
2. Text processed for intent and entities
3. Planning module creates action sequence
4. Navigation and manipulation tasks executed
5. Feedback loop updates AI with execution results

## Validation Requirements

### Message Schema Validation
- All messages follow ROS 2 message conventions
- Required fields are always populated
- Data types are consistent with ROS 2 standards
- Timestamps are properly synchronized

### Performance Requirements
- Real-time message processing (100Hz minimum for control)
- Low latency for safety-critical messages (<10ms)
- Appropriate message compression for bandwidth efficiency
- Efficient serialization for high-frequency data

### Reliability Requirements
- Message validation to prevent invalid data propagation
- Fallback mechanisms for missing or corrupted data
- Error reporting and recovery procedures
- Consistency checks across related message types