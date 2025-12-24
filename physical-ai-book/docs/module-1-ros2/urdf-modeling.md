---
sidebar_position: 4
---

# Humanoid Robot Modeling with URDF

## Introduction

URDF (Unified Robot Description Format) is an XML format used to describe robot models in ROS. It defines the robot's physical structure, including links (rigid bodies) and joints (constraints between links). For humanoid robots, URDF is essential for simulation, visualization, and control.

## URDF Basics

A URDF file describes a robot as a collection of links connected by joints. The structure typically follows this pattern:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="base_to_upper_body" type="fixed">
    <parent link="base_link"/>
    <child link="upper_body"/>
    <origin xyz="0 0 0.1"/>
  </joint>

  <link name="upper_body">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
    </inertial>
  </link>
</robot>
```

## Link Elements

Each link represents a rigid body and contains:

- **Visual**: How the link appears in visualization
- **Collision**: How the link interacts in collision detection
- **Inertial**: Physical properties for dynamics simulation

### Visual Element

The visual element defines how the link appears:

```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.1 0.1"/>
    <!-- or -->
    <cylinder radius="0.05" length="0.2"/>
    <!-- or -->
    <sphere radius="0.05"/>
    <!-- or -->
    <mesh filename="package://my_robot/meshes/link.stl"/>
  </geometry>
  <material name="my_material">
    <color rgba="0.8 0.2 0.2 1.0"/>
  </material>
</visual>
```

### Collision Element

The collision element defines the collision properties:

```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.1 0.1"/>
  </geometry>
</collision>
```

### Inertial Element

The inertial element defines the mass properties:

```xml
<inertial>
  <mass value="0.1"/>
  <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
</inertial>
```

## Joint Elements

Joints connect links and define how they can move relative to each other:

### Fixed Joint
```xml
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link"/>
  <child link="child_link"/>
</joint>
```

### Revolute Joint
```xml
<joint name="revolute_joint" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
</joint>
```

### Continuous Joint
```xml
<joint name="continuous_joint" type="continuous">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit effort="10" velocity="1"/>
</joint>
```

## Xacro for Complex Models

Xacro (XML Macros) allows you to use macros and variables to simplify complex URDF files:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="wheel_radius" value="0.0375" />
  <xacro:property name="wheel_width" value="0.025" />

  <!-- Macro for creating a wheel -->
  <xacro:macro name="wheel" params="prefix parent xyz rpy">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel_link"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="0 1 0"/>
    </joint>

    <link name="${prefix}_wheel_link">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.1"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.002"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:wheel prefix="front_left" parent="base_link" xyz="0.1 0.1 0" rpy="0 0 0"/>
  <xacro:wheel prefix="front_right" parent="base_link" xyz="0.1 -0.1 0" rpy="0 0 0"/>

</robot>
```

## Complete Humanoid Robot Example

Here's a simplified example of a humanoid robot structure:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_humanoid">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.1"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.15 0.3"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.15 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="torso_to_head" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.2"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.1 0.1"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.2"/>
      </geometry>
      <origin xyz="0 0 0.1" rpy="1.57 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.2"/>
      </geometry>
      <origin xyz="0 0 0.1" rpy="1.57 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>

</robot>
```

## Validation and Testing

To validate your URDF model:

1. **Parse Check**: Use `check_urdf` to validate syntax:
   ```bash
   check_urdf /path/to/your/robot.urdf
   ```

2. **Visualization**: Use RViz or Gazebo to visualize the model:
   ```bash
   ros2 run rviz2 rviz2
   ```

3. **Joint State Publisher**: Use joint_state_publisher for visualization:
   ```bash
   ros2 run joint_state_publisher joint_state_publisher
   ```

## Conclusion

URDF modeling is fundamental for humanoid robotics as it provides the geometric and physical representation of the robot. Proper modeling ensures accurate simulation and effective control. In the next module, we'll explore how to create digital twins of these robots in simulation environments.