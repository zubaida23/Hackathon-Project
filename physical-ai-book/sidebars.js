module.exports = {
  docs: [
    {
      type: 'doc',
      id: 'intro',
    },
    {
      type: 'category',
      label: 'Module 1: ROS2 - The Robotic Nervous System',
      items: [
        'module-1-ros2/intro',
        'module-1-ros2/ros2-basics',
        'module-1-ros2/rclpy-agents',
        'module-1-ros2/urdf-humanoid'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin - Virtual Reality Bridge',
      items: [
        'module-2-digital-twin/intro',
        'module-2-digital-twin/gazebo-simulation',
        'module-2-digital-twin/sensors',
        'module-2-digital-twin/unity-visualization'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Isaac - AI-Powered Robot Brain',
      items: [
        'module-3-isaac/intro',
        'module-3-isaac/isaac-sim',
        'module-3-isaac/isaac-ros',
        'module-3-isaac/navigation'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA - Voice-Enabled AI Robotics',
      items: [
        'module-4-vla/intro',
        'module-4-vla/whisper-voice',
        'module-4-vla/llm-planning',
        'module-4-vla/capstone'
      ],
    },
  ],
};