---
sidebar_position: 2
---

# API Documentation Reference

## ROS 2 Client Libraries

### rclpy (Python)

The `rclpy` package provides Python bindings for ROS 2.

#### Basic Node Creation

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('Node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

#### Publishers and Subscribers

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerListenerNode(Node):
    def __init__(self):
        super().__init__('talker_listener')

        # Publisher
        self.publisher = self.create_publisher(String, 'topic', 10)

        # Subscriber
        self.subscriber = self.create_subscription(
            String, 'topic', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')
```

#### Services and Clients

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response
```

## Isaac ROS Components

### Perception Pipeline Components

- `isaac_ros_apriltag` - AprilTag detection
- `isaac_ros_detectnet` - Object detection
- `isaac_ros_hawks` - Stereo depth estimation
- `isaac_ros_pointcloud` - Point cloud processing

### Navigation Components

- `isaac_ros_goal_pose` - Goal pose setting
- `isaac_ros_path_planner` - Path planning
- `isaac_ros_localization` - Robot localization

## Gazebo Integration

### Gazebo Plugins

- `libgazebo_ros_control.so` - ROS control interface
- `libgazebo_ros_imu.so` - IMU sensor interface
- `libgazebo_ros_camera.so` - Camera sensor interface
- `libgazebo_ros_ray_sensor.so` - LiDAR/Ray sensor interface