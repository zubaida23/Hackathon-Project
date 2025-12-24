#!/usr/bin/env python3

"""
Module 1: The Robotic Nervous System
Sensor Publisher Node

This node simulates sensor data publication for the humanoid robot.
It publishes various sensor messages including joint states, IMU data, and other sensor readings.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Vector3
import math
import random


class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # Create publishers for different sensor types
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)

        # Timer for publishing sensor data
        self.timer = self.create_timer(0.1, self.publish_sensor_data)  # 10 Hz

        # Initialize joint names for a humanoid robot
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint', 'left_wrist_joint',
            'right_shoulder_joint', 'right_elbow_joint', 'right_wrist_joint',
            'head_joint'
        ]

        self.get_logger().info('Sensor Publisher node initialized')

    def publish_sensor_data(self):
        """Publish simulated sensor data"""
        # Publish joint states
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = 'base_link'
        joint_msg.name = self.joint_names

        # Simulate joint positions (oscillating for demonstration)
        joint_msg.position = []
        current_time = self.get_clock().now().nanoseconds / 1e9
        for i, _ in enumerate(self.joint_names):
            # Create different oscillation patterns for each joint
            position = math.sin(current_time + i) * 0.5
            joint_msg.position.append(position)

        # Simulate velocities and efforts
        joint_msg.velocity = [random.uniform(-0.1, 0.1) for _ in self.joint_names]
        joint_msg.effort = [random.uniform(-1.0, 1.0) for _ in self.joint_names]

        self.joint_state_publisher.publish(joint_msg)

        # Publish IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Simulate orientation (unit quaternion for now)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0

        # Simulate angular velocity
        imu_msg.angular_velocity.x = random.uniform(-0.1, 0.1)
        imu_msg.angular_velocity.y = random.uniform(-0.1, 0.1)
        imu_msg.angular_velocity.z = random.uniform(-0.1, 0.1)

        # Simulate linear acceleration
        imu_msg.linear_acceleration.x = random.uniform(-9.8, 9.8)
        imu_msg.linear_acceleration.y = random.uniform(-9.8, 9.8)
        imu_msg.linear_acceleration.z = random.uniform(-9.8, 9.8)

        self.imu_publisher.publish(imu_msg)

        self.get_logger().debug('Published sensor data')


def main(args=None):
    rclpy.init(args=args)

    sensor_publisher = SensorPublisher()

    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()