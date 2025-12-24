---
sidebar_position: 3
---

# Sensor Simulation in Digital Twins

## Introduction

Sensor simulation is critical for creating realistic digital twins. In this module, we'll explore how to implement various sensor types in Gazebo, including LiDAR, cameras, and IMUs. Proper sensor simulation enables testing of perception algorithms in a safe environment.

## LiDAR Simulation

LiDAR sensors provide 2D or 3D distance measurements. Here's how to simulate a LiDAR sensor in Gazebo:

```xml
<gazebo reference="laser_link">
  <sensor name="laser" type="ray">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.570796</min_angle>  <!-- -90 degrees -->
          <max_angle>1.570796</max_angle>   <!-- 90 degrees -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/humanoid_robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

### LiDAR ROS 2 Integration

To work with LiDAR data in ROS 2:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            '/humanoid_robot/scan',
            self.lidar_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def lidar_callback(self, msg):
        # Process LiDAR data
        ranges = np.array(msg.ranges)

        # Remove invalid measurements
        valid_ranges = ranges[np.isfinite(ranges)]

        # Example: Find minimum distance
        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            self.get_logger().info(f'Minimum obstacle distance: {min_distance:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    lidar_processor = LidarProcessor()
    rclpy.spin(lidar_processor)
    lidar_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Camera Simulation

Camera sensors provide visual data for perception algorithms:

```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>30</update_rate>
    <camera name="head">
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/humanoid_robot</namespace>
        <remapping>~/image_raw:=camera/image_raw</remapping>
        <remapping>~/image_info:=camera/camera_info</remapping>
      </ros>
      <camera_name>camera</camera_name>
      <frame_name>camera_optical_frame</frame_name>
      <hack_baseline>0.07</hack_baseline>
    </plugin>
  </sensor>
</gazebo>
```

### Camera Data Processing

Processing camera data in ROS 2:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
        self.subscription = self.create_subscription(
            Image,
            '/humanoid_robot/camera/image_raw',
            self.camera_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def camera_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Example: Apply simple image processing
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Example: Detect edges
        edges = cv2.Canny(gray, 50, 150)

        # Optional: Display the processed image
        # cv2.imshow('Camera View', cv_image)
        # cv2.imshow('Edges', edges)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    camera_processor = CameraProcessor()
    rclpy.spin(camera_processor)
    camera_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## IMU Simulation

Inertial Measurement Units (IMUs) provide orientation and acceleration data:

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.003</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.003</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.003</bias_stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <ros>
        <namespace>/humanoid_robot</namespace>
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
      <body_name>imu_link</body_name>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Data Processing

Working with IMU data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math

class ImuProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        self.subscription = self.create_subscription(
            Imu,
            '/humanoid_robot/imu/data',
            self.imu_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def imu_callback(self, msg):
        # Extract orientation (in quaternion format)
        orientation_q = msg.orientation
        # Convert quaternion to Euler angles
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )

        # Extract angular velocity
        angular_velocity = msg.angular_velocity
        # Extract linear acceleration
        linear_acceleration = msg.linear_acceleration

        self.get_logger().info(
            f'Orientation - Roll: {math.degrees(roll):.2f}°, '
            f'Pitch: {math.degrees(pitch):.2f}°, '
            f'Yaw: {math.degrees(yaw):.2f}°'
        )

    def quaternion_to_euler(self, x, y, z, w):
        # Convert quaternion to Euler angles
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    imu_processor = ImuProcessor()
    rclpy.spin(imu_processor)
    imu_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Sensor Fusion

Combining data from multiple sensors for better perception:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Pose
import numpy as np

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # Subscribe to multiple sensors
        self.lidar_subscription = self.create_subscription(
            LaserScan, '/humanoid_robot/scan', self.lidar_callback, 10)
        self.imu_subscription = self.create_subscription(
            Imu, '/humanoid_robot/imu/data', self.imu_callback, 10)

        # Publisher for fused data
        self.pose_publisher = self.create_publisher(Pose, '/humanoid_robot/pose', 10)

        # Initialize sensor data storage
        self.lidar_data = None
        self.imu_data = None

    def lidar_callback(self, msg):
        self.lidar_data = msg
        self.fuse_sensors()

    def imu_callback(self, msg):
        self.imu_data = msg
        self.fuse_sensors()

    def fuse_sensors(self):
        if self.lidar_data and self.imu_data:
            # Example: Combine IMU orientation with LiDAR position data
            # In practice, this would use more sophisticated fusion algorithms
            pose = Pose()

            # Use IMU for orientation
            pose.orientation = self.imu_data.orientation

            # Publish the fused pose
            self.pose_publisher.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    sensor_fusion = SensorFusion()
    rclpy.spin(sensor_fusion)
    sensor_fusion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Conclusion

Sensor simulation is a critical component of digital twin technology. Properly configured sensors provide realistic data that allows for thorough testing of perception and navigation algorithms. In the next module, we'll explore how to implement AI-powered perception and navigation systems using these simulated sensors.