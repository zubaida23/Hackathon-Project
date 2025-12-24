---
sidebar_position: 2
---

# Gazebo Setup and Simulation Environments

## Introduction

Gazebo is a powerful 3D simulation environment that provides physics simulation, realistic sensor models, and rendering capabilities. It's essential for testing robotic algorithms in a safe, repeatable environment before deploying to real hardware.

## Installing and Setting Up Gazebo

For ROS 2 Humble, we'll use Gazebo Garden or Fortress. Install Gazebo alongside ROS 2:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-*
```

## Creating a Basic Gazebo World

Gazebo worlds are defined in SDF (Simulation Description Format) files. Here's a basic world file:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Include a default ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a default light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Define a simple room -->
    <model name="simple_room">
      <pose>0 0 0 0 0 0</pose>
      <static>true</static>

      <!-- Floor -->
      <link name="floor">
        <visual name="floor_visual">
          <geometry>
            <box>
              <size>10 10 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
        <collision name="floor_collision">
          <geometry>
            <box>
              <size>10 10 0.1</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>1e10</mass>
          <inertia>
            <ixx>1e10</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1e10</iyy>
            <iyz>0</iyz>
            <izz>1e10</izz>
          </inertia>
        </inertial>
      </link>

      <!-- Walls -->
      <link name="wall1">
        <pose>5 0 2.5 0 0 0</pose>
        <visual name="wall1_visual">
          <geometry>
            <box>
              <size>0.1 10 5</size>
            </box>
          </geometry>
        </visual>
        <collision name="wall1_collision">
          <geometry>
            <box>
              <size>0.1 10 5</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>1e10</mass>
          <inertia>
            <ixx>1e10</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1e10</iyy>
            <iyz>0</iyz>
            <izz>1e10</izz>
          </inertia>
        </inertial>
      </link>

      <!-- Add more walls as needed -->
    </model>
  </world>
</sdf>
```

## Launching Gazebo with ROS 2

To launch Gazebo with ROS 2, create a launch file:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    world_file = LaunchConfiguration('world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='worlds/empty.sdf',
            description='Choose one of the world files from `/gazebo_ros_pkgs/gazebo_worlds`'
        ),

        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        )
    ])
```

## Integrating Your Robot Model

To spawn your robot in Gazebo, use the `spawn_entity` service:

```python
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import os

class RobotSpawner(Node):
    def __init__(self):
        super().__init__('robot_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.spawn_robot()

    def spawn_robot(self):
        # Load the robot model from URDF
        urdf_path = os.path.join(os.path.dirname(__file__), 'robot.urdf')
        with open(urdf_path, 'r') as urdf_file:
            urdf_model = urdf_file.read()

        # Create the request
        req = SpawnEntity.Request()
        req.name = 'humanoid_robot'
        req.xml = urdf_model
        req.initial_pose.position.x = 0.0
        req.initial_pose.position.y = 0.0
        req.initial_pose.position.z = 1.0
        req.initial_pose.orientation.x = 0.0
        req.initial_pose.orientation.y = 0.0
        req.initial_pose.orientation.z = 0.0
        req.initial_pose.orientation.w = 1.0

        # Call the service
        self.future = self.cli.call_async(req)
        self.get_logger().info('Robot spawn request sent')

def main(args=None):
    rclpy.init(args=args)
    spawner = RobotSpawner()
    rclpy.spin(spawner)
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Physics Configuration

Gazebo's physics engine can be configured for different simulation requirements:

```xml
<physics name="1ms" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## Advanced Gazebo Features

### Plugins

Gazebo supports plugins to extend functionality:

```xml
<model name="humanoid_robot">
  <!-- Include your URDF model here -->

  <!-- ROS 2 Control plugin -->
  <gazebo>
    <plugin name="ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid_robot</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
  </gazebo>

  <!-- IMU sensor plugin -->
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
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
    </sensor>
  </gazebo>
</model>
```

## Performance Optimization

For optimal simulation performance:

1. **Adjust Time Step**: Smaller steps for accuracy, larger for performance
2. **Reduce Real-time Factor**: Use &lt;1.0 for faster than real-time simulation
3. **Optimize Models**: Simplify collision meshes where possible
4. **Limit Update Rates**: Set appropriate update rates for sensors

## Conclusion

Gazebo provides a powerful simulation environment for testing robotic algorithms. Proper configuration ensures realistic physics simulation while maintaining performance. In the next section, we'll explore how to implement realistic sensor simulation in Gazebo.