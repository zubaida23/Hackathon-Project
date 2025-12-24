#!/usr/bin/env python3

"""
Module 1: The Robotic Nervous System
Command Interpreter Node

This node interprets high-level commands and converts them to robot-specific commands.
It serves as the interface between AI agents and the robot controller.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json


class CommandInterpreter(Node):
    def __init__(self):
        super().__init__('command_interpreter')

        # Create subscriber for high-level commands
        self.high_level_command_subscriber = self.create_subscription(
            String,
            'high_level_command',
            self.high_level_command_callback,
            10
        )

        # Create publishers for different command types
        self.robot_command_publisher = self.create_publisher(
            String,
            'robot_command',
            10
        )

        self.velocity_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Publisher for command status/feedback
        self.feedback_publisher = self.create_publisher(
            String,
            'command_feedback',
            10
        )

        self.get_logger().info('Command Interpreter node initialized')

    def high_level_command_callback(self, msg):
        """Process high-level commands from AI agents"""
        try:
            # Parse the command - could be a simple string or JSON with parameters
            command_text = msg.data.strip()

            self.get_logger().info(f'Received high-level command: {command_text}')

            # Determine command type and convert to robot-specific commands
            if command_text.upper() == "STAND UP":
                self.execute_stand_up()
            elif command_text.upper() == "SIT DOWN":
                self.execute_sit_down()
            elif command_text.upper() == "WALK FORWARD":
                self.execute_walk_forward()
            elif command_text.upper() == "WALK BACKWARD":
                self.execute_walk_backward()
            elif command_text.upper() == "TURN LEFT":
                self.execute_turn_left()
            elif command_text.upper() == "TURN RIGHT":
                self.execute_turn_right()
            elif command_text.upper() == "HOME POSITION":
                self.execute_home_position()
            elif command_text.upper() == "SHUTDOWN":
                self.execute_shutdown()
            elif command_text.startswith("MOVE TO"):
                self.execute_move_to(command_text)
            elif command_text.startswith("GRASP"):
                self.execute_grasp(command_text)
            else:
                # Try to parse as JSON command for more complex instructions
                try:
                    command_json = json.loads(command_text)
                    self.process_json_command(command_json)
                except json.JSONDecodeError:
                    self.get_logger().warn(f'Unknown command: {command_text}')
                    self.send_feedback(f"Unknown command: {command_text}")

        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')
            self.send_feedback(f"Error processing command: {e}")

    def execute_stand_up(self):
        """Execute stand up command"""
        self.get_logger().info('Executing stand up command')
        command_msg = String()
        command_msg.data = "STAND_UP"
        self.robot_command_publisher.publish(command_msg)
        self.send_feedback("Standing up")

    def execute_sit_down(self):
        """Execute sit down command"""
        self.get_logger().info('Executing sit down command')
        command_msg = String()
        command_msg.data = "SIT_DOWN"  # This would need to be implemented in controller
        self.robot_command_publisher.publish(command_msg)
        self.send_feedback("Sitting down")

    def execute_walk_forward(self):
        """Execute walk forward command"""
        self.get_logger().info('Executing walk forward command')
        twist_msg = Twist()
        twist_msg.linear.x = 0.5  # Forward velocity
        self.velocity_publisher.publish(twist_msg)
        self.send_feedback("Walking forward")

    def execute_walk_backward(self):
        """Execute walk backward command"""
        self.get_logger().info('Executing walk backward command')
        twist_msg = Twist()
        twist_msg.linear.x = -0.5  # Backward velocity
        self.velocity_publisher.publish(twist_msg)
        self.send_feedback("Walking backward")

    def execute_turn_left(self):
        """Execute turn left command"""
        self.get_logger().info('Executing turn left command')
        twist_msg = Twist()
        twist_msg.angular.z = 0.5  # Left turn
        self.velocity_publisher.publish(twist_msg)
        self.send_feedback("Turning left")

    def execute_turn_right(self):
        """Execute turn right command"""
        self.get_logger().info('Executing turn right command')
        twist_msg = Twist()
        twist_msg.angular.z = -0.5  # Right turn
        self.velocity_publisher.publish(twist_msg)
        self.send_feedback("Turning right")

    def execute_home_position(self):
        """Execute home position command"""
        self.get_logger().info('Executing home position command')
        command_msg = String()
        command_msg.data = "HOME_POSITION"
        self.robot_command_publisher.publish(command_msg)
        self.send_feedback("Moving to home position")

    def execute_shutdown(self):
        """Execute shutdown command"""
        self.get_logger().info('Executing shutdown command')
        command_msg = String()
        command_msg.data = "SHUTDOWN"
        self.robot_command_publisher.publish(command_msg)
        self.send_feedback("Shutting down")

    def execute_move_to(self, command_text):
        """Execute move to location command"""
        self.get_logger().info(f'Executing move to command: {command_text}')
        # This is a simplified version - in practice, this would parse coordinates
        # and plan a path to the destination
        self.send_feedback(f"Moving to location: {command_text[7:]}")  # Remove "MOVE TO"

    def execute_grasp(self, command_text):
        """Execute grasp object command"""
        self.get_logger().info(f'Executing grasp command: {command_text}')
        # This would involve complex manipulation planning
        self.send_feedback(f"Attempting to grasp: {command_text[6:]}")  # Remove "GRASP"

    def process_json_command(self, command_json):
        """Process complex commands in JSON format"""
        command_type = command_json.get('type', '').upper()

        if command_type == "JOINT_MOVE":
            joint_name = command_json.get('joint')
            position = command_json.get('position')
            if joint_name and position is not None:
                command_msg = String()
                command_msg.data = f"MOVE_JOINT {joint_name} {position}"
                self.robot_command_publisher.publish(command_msg)
                self.send_feedback(f"Moving {joint_name} to {position}")
        elif command_type == "TRAJECTORY":
            trajectory_points = command_json.get('points', [])
            self.execute_trajectory(trajectory_points)
        else:
            self.get_logger().warn(f'Unknown JSON command type: {command_type}')
            self.send_feedback(f"Unknown JSON command type: {command_type}")

    def execute_trajectory(self, points):
        """Execute a trajectory of joint positions"""
        self.get_logger().info(f'Executing trajectory with {len(points)} points')
        self.send_feedback(f"Executing trajectory with {len(points)} points")
        # This would implement trajectory following logic

    def send_feedback(self, message):
        """Send command execution feedback"""
        feedback_msg = String()
        feedback_msg.data = message
        self.feedback_publisher.publish(feedback_msg)


def main(args=None):
    rclpy.init(args=args)

    command_interpreter = CommandInterpreter()

    try:
        rclpy.spin(command_interpreter)
    except KeyboardInterrupt:
        command_interpreter.get_logger().info('Command Interpreter interrupted by user')
    finally:
        command_interpreter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()