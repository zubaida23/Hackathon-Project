---
sidebar_position: 3
---

# Simulation Parameters Reference

## Gazebo Physics Parameters

### Global Physics Settings

```xml
<physics name="default" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
</physics>
```

#### Parameter Descriptions

- `max_step_size`: Maximum simulation time step (seconds)
- `real_time_factor`: Target simulation speed relative to real time
- `real_time_update_rate`: Updates per second
- `gravity`: Gravitational acceleration vector (m/s²)

### ODE Solver Parameters

```xml
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
```

## Sensor Configuration Parameters

### Camera Sensor

```xml
<sensor name="camera" type="camera">
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
  </camera>
</sensor>
```

### LiDAR Sensor

```xml
<sensor name="laser" type="ray">
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
</sensor>
```

### IMU Sensor

```xml
<sensor name="imu_sensor" type="imu">
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
```

## Robot Control Parameters

### Joint Limits

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
</joint>
```

### Transmission Parameters

```xml
<transmission name="transmission_joint">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="joint_name">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="actuator_name">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Performance Optimization Settings

### For Real-time Simulation

- Set `real_time_factor` to 1.0 for real-time performance
- Reduce `max_step_size` for accuracy (increases computation)
- Increase `max_step_size` for performance (decreases accuracy)

### For Fast Simulation

- Set `real_time_factor` > 1.0 for faster than real-time
- Use simpler collision meshes
- Reduce sensor update rates
- Limit physics solver iterations