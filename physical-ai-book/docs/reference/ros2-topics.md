---
sidebar_position: 1
---

# ROS 2 Topics Reference

## Common ROS 2 Topics

This reference provides a comprehensive list of common ROS 2 topics used in humanoid robotics applications.

### Navigation Topics

- `/cmd_vel` - Velocity commands for robot movement
- `/move_base_simple/goal` - Simple navigation goal
- `/local_costmap/costmap` - Local costmap for navigation
- `/global_costmap/costmap` - Global costmap for navigation

### Sensor Topics

- `/camera/image_raw` - Raw camera image data
- `/scan` - LiDAR scan data
- `/imu/data` - IMU sensor data
- `/joint_states` - Robot joint positions and velocities

### Robot Control Topics

- `/joint_trajectory` - Joint trajectory commands
- `/tf` - Transform data between coordinate frames
- `/tf_static` - Static transform data
- `/robot_description` - Robot model description

## Message Types

### Common Message Types

- `geometry_msgs/Twist` - Linear and angular velocity commands
- `sensor_msgs/JointState` - Joint position, velocity, and effort
- `sensor_msgs/Image` - Image data from cameras
- `sensor_msgs/LaserScan` - LiDAR scan data
- `nav_msgs/Path` - Navigation path data
- `geometry_msgs/PoseStamped` - Stamped pose data