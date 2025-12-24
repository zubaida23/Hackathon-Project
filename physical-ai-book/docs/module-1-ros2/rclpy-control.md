---
sidebar_position: 3
---

# Python-based Robot Control with rclpy

## Introduction

rclpy is the Python client library for ROS 2. It provides the interface between Python programs and the ROS 2 middleware, enabling you to create nodes, publish/subscribe to topics, provide/call services, and interact with actions.

## Setting Up Your Environment

Before creating robot controllers with rclpy, ensure you have:

- ROS 2 Humble Hawksbill installed
- Python 3.8 or higher
- Basic understanding of ROS 2 concepts

## Creating a Robot Controller Node

Let's create a basic robot controller that can command joint positions:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publisher for joint commands
        self.joint_cmd_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            10
        )

        # Subscriber for current joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Initialize joint states
        self.current_joint_positions = {}
        self.target_joint_positions = {}

        self.get_logger().info('Robot Controller initialized')

    def joint_state_callback(self, msg):
        """Callback to update current joint states"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def control_loop(self):
        """Main control loop"""
        # Example: Move joints to target positions
        self.move_to_target_positions()

    def move_to_target_positions(self):
        """Send joint trajectory command"""
        msg = JointTrajectory()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Define joint names
        msg.joint_names = ['joint1', 'joint2', 'joint3']  # Replace with actual joint names

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0]  # Target positions
        point.velocities = [0.0, 0.0, 0.0]  # Target velocities
        point.time_from_start.sec = 1  # 1 second to reach target
        point.time_from_start.nanosec = 0

        msg.points = [point]

        # Publish the command
        self.joint_cmd_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## AI-to-Controller Communication

To enable AI agents to communicate with robot controllers, we can create a service that accepts high-level commands:

```python
from example_interfaces.srv import Trigger
import rclpy
from rclpy.node import Node

class AIControllerInterface(Node):
    def __init__(self):
        super().__init__('ai_controller_interface')

        # Service to receive commands from AI
        self.command_service = self.create_service(
            Trigger,
            'ai_robot_command',
            self.command_callback
        )

        # Publisher to send commands to robot controller
        self.command_publisher = self.create_publisher(
            String,
            'robot_commands',
            10
        )

        self.get_logger().info('AI Controller Interface initialized')

    def command_callback(self, request, response):
        """Handle commands from AI agent"""
        self.get_logger().info('Received command from AI')

        # Process the command and send to robot
        # This is where AI logic would translate high-level goals to robot actions
        response.success = True
        response.message = 'Command received and processed'

        return response

def main(args=None):
    rclpy.init(args=args)
    ai_interface = AIControllerInterface()

    try:
        rclpy.spin(ai_interface)
    except KeyboardInterrupt:
        pass
    finally:
        ai_interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices

1. **Error Handling**: Always implement proper error handling for robot control
2. **Safety**: Implement safety checks and limits for joint positions and velocities
3. **State Management**: Keep track of robot state to ensure consistent behavior
4. **Logging**: Use appropriate logging levels for debugging and monitoring

## Conclusion

In this section, we've explored how to create robot controllers using rclpy and how to establish communication between AI agents and robot controllers. In the next section, we'll learn about modeling humanoid robots with URDF.