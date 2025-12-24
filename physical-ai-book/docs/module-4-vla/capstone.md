---
sidebar_position: 4
---

# Capstone: Autonomous Humanoid Robot (VLA System)

## Introduction

The capstone project integrates all components learned throughout the course into a complete Vision-Language-Action (VLA) system. This module demonstrates how to combine ROS 2 communication, digital twin simulation, AI perception and navigation, and voice-activated control into a unified autonomous humanoid robot system.

## System Architecture

The complete VLA system architecture:

```
┌─────────────────────────────────────────────────────────────────┐
│                    HUMAN-ROBOT INTERACTION                     │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │   Voice Input   │  │  Natural Lang │  │   Visual Input  │ │
│  │   Processing    │  │   Processing  │  │   Processing    │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘ │
└─────────────┬─────────────────┬──────────────────┬─────────────┘
              │                 │                  │
              ▼                 ▼                  ▼
    ┌─────────────────────────────────────────────────────────┐
    │                   COGNITIVE PLANNING                   │
    │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────┐ │
    │  │   LLM Planner   │  │  Task Manager   │  │ Safety  │ │
    │  │                 │  │                 │  │ Checker │ │
    │  └─────────────────┘  └─────────────────┘  └─────────┘ │
    └─────────────────────────┬───────────────────────────────┘
                              │
                              ▼
    ┌─────────────────────────────────────────────────────────┐
    │                  ACTION EXECUTION                      │
    │  ┌───────────────┐  ┌─────────────────┐  ┌──────────┐ │
    │  │   Navigation  │  │  Manipulation   │  │  Speech  │ │
    │  │               │  │                 │  │  Output  │ │
    │  └───────────────┘  └─────────────────┘  └──────────┘ │
    └─────────────────────────┬───────────────────────────────┘
                              │
                              ▼
    ┌─────────────────────────────────────────────────────────┐
    │                   ROBOT CONTROL                        │
    │  ┌───────────────┐  ┌─────────────────┐  ┌──────────┐ │
    │  │   ROS 2 Core  │  │  Simulation/    │  │ Hardware │ │
    │  │   Framework   │  │  Real Robot     │  │  Layer   │ │
    │  └───────────────┘  └─────────────────┘  └──────────┘ │
    └─────────────────────────────────────────────────────────┘
```

## Complete VLA System Implementation

Here's the complete implementation that ties all modules together:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Image, LaserScan
import json
import threading
import time

class VLACapstoneSystem(Node):
    def __init__(self):
        super().__init__('vla_capstone_system')

        # Initialize subsystems
        self.voice_processor = VoiceProcessor(self)
        self.llm_planner = LLMPlanner(self)
        self.action_executor = ActionExecutor(self)

        # Publishers
        self.status_pub = self.create_publisher(String, '/vla_system/status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/high_level_command', self.command_callback, 10)

        # System state
        self.system_active = True
        self.current_task = None
        self.robot_state = {
            'location': 'LIVING_ROOM',
            'holding_object': None,
            'battery_level': 100.0
        }

        # Start system monitoring
        self.status_timer = self.create_timer(1.0, self.system_status_monitor)

        self.get_logger().info('VLA Capstone System initialized')

    def command_callback(self, msg):
        """Main command processing entry point"""
        command = msg.data
        self.get_logger().info(f'Received high-level command: {command}')

        # Update system status
        status_msg = String()
        status_msg.data = f'PROCESSING: {command}'
        self.status_pub.publish(status_msg)

        # Process through the VLA pipeline
        try:
            # 1. Plan the task using LLM
            plan = self.llm_planner.generate_plan(command)

            if plan:
                # 2. Execute the plan
                self.action_executor.execute_plan(plan)

                # 3. Update robot state based on execution
                self.update_robot_state(plan)

                # 4. Report completion
                completion_msg = String()
                completion_msg.data = f'COMPLETED: {command}'
                self.status_pub.publish(completion_msg)

                self.get_logger().info(f'Task completed: {command}')
            else:
                # Report failure
                failure_msg = String()
                failure_msg.data = f'FAILED: Could not generate plan for {command}'
                self.status_pub.publish(failure_msg)

                self.get_logger().error(f'Failed to generate plan for: {command}')

        except Exception as e:
            self.get_logger().error(f'Error processing command {command}: {e}')
            error_msg = String()
            error_msg.data = f'ERROR: {str(e)}'
            self.status_pub.publish(error_msg)

    def update_robot_state(self, plan):
        """Update robot state based on executed plan"""
        for action in plan:
            action_type = action.get('action', '')

            if action_type == 'PICK_UP_OBJECT':
                obj = action.get('parameters', {}).get('object', '')
                self.robot_state['holding_object'] = obj
                self.get_logger().info(f'Robot now holding: {obj}')

            elif action_type == 'PLACE_OBJECT':
                self.robot_state['holding_object'] = None
                self.get_logger().info('Robot is no longer holding an object')

            elif action_type.startswith('NAVIGATE_TO'):
                location = action.get('parameters', {}).get('location', '')
                if location:
                    self.robot_state['location'] = location
                    self.get_logger().info(f'Robot moved to: {location}')

    def system_status_monitor(self):
        """Monitor and report system status"""
        status = {
            'active': self.system_active,
            'current_task': self.current_task,
            'robot_state': self.robot_state,
            'timestamp': self.get_clock().now().seconds_nanoseconds()
        }

        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)

class VoiceProcessor:
    def __init__(self, node):
        self.node = node
        self.node.create_subscription(
            String, '/voice/text', self.voice_callback, 10)

    def voice_callback(self, msg):
        """Process voice input"""
        text = msg.data
        self.node.get_logger().info(f'Voice input: {text}')

        # Convert voice to high-level command
        command = self.process_voice_command(text)

        # Publish as high-level command
        cmd_msg = String()
        cmd_msg.data = command
        self.node.create_publisher(String, '/high_level_command', 10).publish(cmd_msg)

    def process_voice_command(self, text):
        """Process voice command text"""
        # In a real system, this would be more sophisticated
        return text

class LLMPlanner:
    def __init__(self, node):
        self.node = node

    def generate_plan(self, command):
        """Generate action plan from command using LLM"""
        # This is a simplified implementation
        # In practice, this would use a real LLM with proper prompting

        plan = []

        if 'kitchen' in command.lower():
            plan.append({
                'action': 'NAVIGATE_TO',
                'parameters': {'location': 'KITCHEN'},
                'description': 'Move to kitchen'
            })

        if 'cup' in command.lower():
            plan.append({
                'action': 'PICK_UP_OBJECT',
                'parameters': {'object': 'CUP'},
                'description': 'Pick up the cup'
            })

        if 'bring' in command.lower() or 'deliver' in command.lower():
            plan.append({
                'action': 'NAVIGATE_TO',
                'parameters': {'location': 'LIVING_ROOM'},
                'description': 'Return to living room'
            })

        if not plan:
            # Default action if no specific plan identified
            plan.append({
                'action': 'SPEAK',
                'parameters': {'text': 'I am not sure how to do that'},
                'description': 'Express uncertainty'
            })

        return plan

class ActionExecutor:
    def __init__(self, node):
        self.node = node
        self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)

    def execute_plan(self, plan):
        """Execute the action plan step by step"""
        for i, action in enumerate(plan):
            self.node.get_logger().info(f'Executing action {i+1}/{len(plan)}: {action["action"]}')

            success = self.execute_action(action)

            if not success:
                self.node.get_logger().error(f'Action failed: {action}')
                break

            # Small delay between actions
            time.sleep(0.5)

    def execute_action(self, action):
        """Execute a single action"""
        action_type = action.get('action', '')

        try:
            if action_type == 'NAVIGATE_TO':
                return self.navigate_to(action)
            elif action_type == 'PICK_UP_OBJECT':
                return self.pick_up_object(action)
            elif action_type == 'PLACE_OBJECT':
                return self.place_object(action)
            elif action_type == 'SPEAK':
                return self.speak(action)
            elif action_type == 'MOVE_FORWARD':
                return self.move_forward()
            elif action_type == 'TURN_LEFT':
                return self.turn_left()
            elif action_type == 'TURN_RIGHT':
                return self.turn_right()
            elif action_type == 'STOP':
                return self.stop()
            else:
                self.node.get_logger().warn(f'Unknown action: {action_type}')
                return False

        except Exception as e:
            self.node.get_logger().error(f'Error executing action {action_type}: {e}')
            return False

    def navigate_to(self, action):
        """Navigate to a specific location"""
        location = action.get('parameters', {}).get('location', '')
        self.node.get_logger().info(f'Navigating to {location}')

        # In a real system, this would use navigation stack
        # For simulation, we'll just move forward for a bit
        twist = Twist()
        twist.linear.x = 0.3  # Move forward at 0.3 m/s
        self.cmd_vel_pub.publish(twist)

        # Wait for a bit to simulate movement
        time.sleep(2)

        # Stop
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

        return True

    def pick_up_object(self, action):
        """Pick up an object"""
        obj = action.get('parameters', {}).get('object', 'object')
        self.node.get_logger().info(f'Attempting to pick up {obj}')

        # In a real system, this would control robot arms
        # For now, just simulate the action
        time.sleep(1)

        return True

    def place_object(self, action):
        """Place an object"""
        self.node.get_logger().info('Placing object')

        # In a real system, this would control robot arms
        # For now, just simulate the action
        time.sleep(1)

        return True

    def speak(self, action):
        """Speak a message"""
        text = action.get('parameters', {}).get('text', 'Hello')
        self.node.get_logger().info(f'Speaking: {text}')

        # In a real system, this would use text-to-speech
        # For now, just log the message
        return True

    def move_forward(self):
        """Move robot forward"""
        twist = Twist()
        twist.linear.x = 0.5
        self.cmd_vel_pub.publish(twist)
        time.sleep(1)
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        return True

    def turn_left(self):
        """Turn robot left"""
        twist = Twist()
        twist.angular.z = 0.5
        self.cmd_vel_pub.publish(twist)
        time.sleep(1)
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        return True

    def turn_right(self):
        """Turn robot right"""
        twist = Twist()
        twist.angular.z = -0.5
        self.cmd_vel_pub.publish(twist)
        time.sleep(1)
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        return True

    def stop(self):
        """Stop robot movement"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        return True

def main(args=None):
    rclpy.init(args=args)
    vla_system = VLACapstoneSystem()

    try:
        rclpy.spin(vla_system)
    except KeyboardInterrupt:
        pass
    finally:
        vla_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch File for the Complete System

Create a launch file to start all components:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        # Voice processing node
        Node(
            package='your_robot_package',
            executable='voice_processor',
            name='voice_processor',
            output='screen'
        ),

        # LLM planner node
        Node(
            package='your_robot_package',
            executable='llm_planner',
            name='llm_planner',
            output='screen'
        ),

        # Action executor node
        Node(
            package='your_robot_package',
            executable='action_executor',
            name='action_executor',
            output='screen'
        ),

        # Main VLA system
        Node(
            package='your_robot_package',
            executable='vla_capstone_system',
            name='vla_capstone_system',
            output='screen'
        ),

        # Navigation stack (if using Nav2)
        Node(
            package='nav2_bringup',
            executable='nav2_bringup',
            name='nav2_bringup',
            output='screen'
        )
    ])
```

## Testing the Complete System

Here's how to test the complete VLA system:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class VLATester(Node):
    def __init__(self):
        super().__init__('vla_tester')

        # Publisher for commands
        self.command_pub = self.create_publisher(String, '/high_level_command', 10)

        # Subscriber for status
        self.status_sub = self.create_subscription(
            String, '/vla_system/status', self.status_callback, 10)

        self.test_commands = [
            "Go to the kitchen and bring me a cup",
            "Move forward",
            "Turn left",
            "Stop"
        ]

        # Start testing after a delay
        self.timer = self.create_timer(3.0, self.run_tests)
        self.current_test = 0

        self.get_logger().info('VLA Tester initialized')

    def status_callback(self, msg):
        """Receive system status updates"""
        self.get_logger().info(f'System status: {msg.data}')

    def run_tests(self):
        """Run a sequence of tests"""
        if self.current_test < len(self.test_commands):
            command = self.test_commands[self.current_test]
            self.get_logger().info(f'Sending test command: {command}')

            cmd_msg = String()
            cmd_msg.data = command
            self.command_pub.publish(cmd_msg)

            self.current_test += 1

            # Schedule next test
            self.timer = self.create_timer(5.0, self.run_tests)
        else:
            self.get_logger().info('All tests completed')

def main(args=None):
    rclpy.init(args=args)
    tester = VLATester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Performance Evaluation

To evaluate the performance of your VLA system:

```python
class VLAPerformanceEvaluator:
    def __init__(self):
        self.metrics = {
            'command_accuracy': 0.0,
            'response_time': 0.0,
            'task_completion_rate': 0.0,
            'safety_violations': 0
        }
        self.test_results = []

    def evaluate_command_accuracy(self, command, expected_action, actual_action):
        """Evaluate how accurately the system interpreted the command"""
        # Compare expected vs actual actions
        if expected_action == actual_action:
            return 1.0  # Perfect match
        else:
            return 0.0  # No match

    def evaluate_task_completion(self, task, success):
        """Evaluate task completion"""
        return 1.0 if success else 0.0

    def evaluate_response_time(self, start_time, end_time):
        """Evaluate system response time"""
        response_time = end_time - start_time
        # Normalize to a 0-1 scale (1.0 is best)
        # Assuming 5 seconds is the maximum acceptable time
        return max(0.0, min(1.0, (5.0 - response_time) / 5.0))

    def calculate_overall_score(self):
        """Calculate overall system performance score"""
        if not self.test_results:
            return 0.0

        total_score = 0.0
        for result in self.test_results:
            total_score += result.get('score', 0.0)

        return total_score / len(self.test_results)
```

## Troubleshooting Common Issues

### Voice Recognition Issues
- **Problem**: Poor voice recognition accuracy
- **Solution**: Improve audio preprocessing, use noise reduction, or adjust microphone sensitivity

### Planning Issues
- **Problem**: LLM generates invalid or unsafe plans
- **Solution**: Add more explicit safety constraints to prompts and implement plan validation

### Execution Issues
- **Problem**: Robot fails to execute planned actions
- **Solution**: Implement robust action monitoring and fallback behaviors

### Integration Issues
- **Problem**: Components don't communicate properly
- **Solution**: Verify ROS 2 message types and topic names match across components

## Conclusion

The VLA capstone system demonstrates the integration of all components learned in this course:

1. **Module 1**: ROS 2 communication and robot control
2. **Module 2**: Simulation and sensor integration
3. **Module 3**: AI perception and navigation
4. **Module 4**: Voice processing and LLM planning

This complete system enables a humanoid robot to understand natural language commands, plan appropriate actions, and execute them safely in its environment. The modular architecture allows for continued improvement of individual components while maintaining system stability.

Congratulations on completing the Physical AI & Humanoid Robotics course! You now have the knowledge to build sophisticated humanoid robot systems that can interact naturally with humans and perform complex tasks autonomously.