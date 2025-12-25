#!/usr/bin/env python3

"""
Module 1: The Robotic Nervous System
Robot Controller Node

This node implements basic robot control functionality using rclpy.
It subscribes to command topics and publishes control messages to the robot.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Create subscribers for command topics
        self.command_subscriber = self.create_subscription(
            String,
            'robot_command',
            self.command_callback,
            10
        )

        self.velocity_command_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_command_callback,
            10
        )

        # Create publishers for control outputs
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            'joint_commands',
            10
        )

        self.status_publisher = self.create_publisher(
            String,
            'robot_status',
            10
        )

        # Timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # Robot state tracking
        self.current_joint_states = None
        self.robot_status = "IDLE"

        # Subscribe to joint states to monitor current state
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('Robot Controller node initialized')

    def command_callback(self, msg):
        """Handle incoming robot commands"""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        # Process different types of commands
        if command == "HOME_POSITION":
            self.move_to_home_position()
        elif command == "STAND_UP":
            self.stand_up_sequence()
        elif command == "SHUTDOWN":
            self.shutdown_sequence()
        elif command.startswith("MOVE_JOINT"):
            # Parse joint movement command: "MOVE_JOINT joint_name position"
            parts = command.split()
            if len(parts) >= 3:
                joint_name = parts[1]
                try:
                    position = float(parts[2])
                    self.move_single_joint(joint_name, position)
                except ValueError:
                    self.get_logger().error(f'Invalid position value in command: {command}')
        else:
            self.get_logger().warn(f'Unknown command: {command}')

        # Update status
        self.robot_status = f"PROCESSED: {command}"

    def velocity_command_callback(self, msg):
        """Handle velocity commands for mobile base"""
        self.get_logger().info(f'Received velocity command: linear={msg.linear}, angular={msg.angular}')

        # In a real robot, this would control the base movement
        # For simulation, we'll just log the command
        self.robot_status = f"VELOCITY_CMD: {msg.linear.x:.2f}, {msg.angular.z:.2f}"

    def joint_state_callback(self, msg):
        """Update current joint states"""
        self.current_joint_states = msg

    def move_to_home_position(self):
        """Move all joints to home position"""
        self.get_logger().info('Moving to home position')

        # Create command to move all joints to zero position
        home_command = Float64MultiArray()
        home_command.data = [0.0] * 13  # Assuming 13 joints based on sensor publisher

        self.joint_command_publisher.publish(home_command)
        self.robot_status = "MOVING_TO_HOME"

    def stand_up_sequence(self):
        """Execute stand up sequence"""
        self.get_logger().info('Executing stand up sequence')

        # This would be a more complex sequence in a real robot
        # For now, we'll just move to a standing position
        stand_command = Float64MultiArray()
        # Simplified standing position (this would be tuned for actual robot)
        stand_positions = [
            0.0, 0.0, 0.0,  # Left leg
            0.0, 0.0, 0.0,  # Right leg
            0.1, -0.5, 0.0,  # Left arm
            0.1, -0.5, 0.0,  # Right arm
            0.0  # Head
        ]
        stand_command.data = stand_positions

        self.joint_command_publisher.publish(stand_command)
        self.robot_status = "STANDING_UP"

    def move_single_joint(self, joint_name, position):
        """Move a single joint to specified position"""
        self.get_logger().info(f'Moving {joint_name} to position {position}')

        # In a real implementation, we would map joint names to indices
        # For now, we'll just send a command with the position at index 0
        command = Float64MultiArray()
        command.data = [position] + [0.0] * 12  # Position for first joint, zeros for others

        self.joint_command_publisher.publish(command)
        self.robot_status = f"MOVING_JOINT: {joint_name}"

    def shutdown_sequence(self):
        """Execute safe shutdown sequence"""
        self.get_logger().info('Executing shutdown sequence')

        # Move to safe position before shutdown
        safe_command = Float64MultiArray()
        safe_command.data = [0.0] * 13  # All joints to zero

        self.joint_command_publisher.publish(safe_command)
        time.sleep(0.5)  # Allow time for movement

        self.robot_status = "SHUTDOWN_COMPLETE"

    def publish_status(self):
        """Publish current robot status"""
        status_msg = String()
        status_msg.data = self.robot_status
        self.status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)

    robot_controller = RobotController()

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        robot_controller.get_logger().info('Robot Controller interrupted by user')
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()