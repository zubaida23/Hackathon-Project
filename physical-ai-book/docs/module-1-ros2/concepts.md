---
sidebar_position: 2
---

# ROS 2 Core Concepts

## Introduction

ROS 2 (Robot Operating System 2) provides the communication infrastructure that enables different components of a robot system to work together. Understanding these core concepts is essential for building robust robotic systems.

## Nodes

A **node** is a process that performs computation. In ROS 2, nodes are the fundamental building blocks of a robot application. Each node can perform specific tasks and communicate with other nodes through topics, services, or actions.

### Creating a Node

Here's a basic example of a ROS 2 node in Python:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from minimal node!')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics

**Topics** enable asynchronous communication between nodes using a publish-subscribe pattern. Multiple nodes can publish to the same topic, and multiple nodes can subscribe to the same topic.

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

## Services

**Services** provide synchronous request-response communication between nodes. A service has a client that sends a request and a server that processes the request and sends back a response.

### Service Definition

Services are defined in `.srv` files with the request and response format:

```
# Request (input)
string name
int32 age
---
# Response (output)
bool success
string message
```

## Actions

**Actions** are used for long-running tasks that may take significant time to complete. They provide feedback during execution and can be canceled.

### Action Definition

Actions are defined in `.action` files:

```
# Goal
int32 order
---
# Result
int32 sequence
---
# Feedback
int32 sequence
```

## Conclusion

Understanding these core ROS 2 concepts is crucial for building effective robotic systems. In the next section, we'll explore how to implement these concepts using Python and rclpy.