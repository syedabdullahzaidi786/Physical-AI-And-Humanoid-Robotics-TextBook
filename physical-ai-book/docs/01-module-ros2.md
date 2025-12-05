---
id: module-ros2
title: "Module 1: Robotic Nervous System (ROS 2)"
sidebar_position: 1
---

# Module 1: Robotic Nervous System (ROS 2)

## Introduction to ROS 2

The Robot Operating System (ROS 2) is a flexible framework for building robot applications. It provides:

- **Publish-Subscribe Messaging**: Nodes communicate via topics (e.g., `/camera/image_raw`, `/imu/data`)
- **Request-Reply Services**: Synchronous communication for queries (e.g., spawn entity, move robot)
- **Parameter Servers**: Dynamic configuration without redeployment
- **Launch Systems**: Orchestrate multiple nodes and launch complex workflows

ROS 2 emphasizes modern C++ and Python, real-time safety, and cross-platform compatibility.

## Core Concepts

### Nodes & Topics

A **node** is an executable process running a specific task. Nodes communicate via **topics**:

```
[Robot State Publisher] --publish--> /tf (Transform Tree)
[Camera Node]         --publish--> /camera/image_raw
[Perception Pipeline] --subscribe--> /camera/image_raw
                      --publish--> /detections
```

### Launch Files

Launch files orchestrate multiple nodes. Example `sim_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_state_publisher', 
             executable='robot_state_publisher',
             parameters=[{'robot_description': robot_urdf}]),
        Node(package='gazebo_ros', 
             executable='gazebo_server',
             arguments=['-s', 'libgazebo_ros_init.so']),
    ])
```

### URDF & XACRO

**URDF** (Unified Robot Description Format) defines robot structure in XML:

```xml
<robot name="humanoid">
  <link name="base_link">
    <inertial>
      <mass value="20"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <joint name="torso_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="torso"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57"/>
  </joint>
</robot>
```

**XACRO** (XML Macros) adds parameterization for template reuse.

## Setting Up a ROS 2 Project

### 1. Create a Package

```bash
ros2 pkg create --build-type ament_python my_robot_pkg
cd my_robot_pkg
```

### 2. Define Dependencies (package.xml)

```xml
<?xml version="1.0"?>
<package format="3">
  <name>robot_description</name>
  <version>0.0.1</version>
  <description>Humanoid robot URDF and launch files</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>robot_state_publisher</depend>
  <depend>gazebo_ros</depend>
</package>
```

### 3. Create Launch & URDF Files

Store URDF files in `urdf/` subdirectory and launch files in `launch/` subdirectory.

## Publishing & Subscribing in Python

### Publisher Example

```python
import rclpy
from sensor_msgs.msg import Image

class CameraPublisher(rclpy.Node):
    def __init__(self):
        super().__init__('camera_pub')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.033, self.timer_callback)  # 30 Hz

    def timer_callback(self):
        msg = Image()
        msg.data = [...]  # image data
        self.pub.publish(msg)

if __name__ == '__main__':
    rclpy.init()
    node = CameraPublisher()
    rclpy.spin(node)
```

### Subscriber Example

```python
from sensor_msgs.msg import Image

class PerceptionNode(rclpy.Node):
    def __init__(self):
        super().__init__('perception')
        self.sub = self.create_subscription(Image, '/camera/image_raw', 
                                            self.image_callback, 10)

    def image_callback(self, msg):
        # Process image
        detections = self.detect_objects(msg.data)
        self.pub.publish(detections)
```

## Gazebo Integration

Gazebo simulates robot physics and sensors. Key plugins:

- **robot_state_publisher**: Publishes `/tf` transform tree
- **gazebo_ros_spawn_entity**: Spawns URDF into Gazebo world
- **gazebo_ros_camera**: Simulates camera sensor with image output

### Spawn Entity via ROS Service

```bash
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity \
  "{name: 'my_robot', xml: $(cat robot.urdf)}"
```

## Best Practices

1. **Modularity**: Each node handles one responsibility (Single Responsibility Principle)
2. **Namespacing**: Use namespaces to avoid topic collisions (`/robot1/camera` vs `/robot2/camera`)
3. **Parameter Tuning**: Use parameters for sensor rates, thresholds, and paths
4. **Error Handling**: Check for node failures and implement recovery logic
5. **Testing**: Unit-test nodes independently before integration

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Topic not found | Check node is running: `ros2 node list` |
| Transform errors | Verify `/tf` tree: `ros2 run tf2_tools view_frames` |
| URDF parse errors | Validate: `check_urdf my_robot.urdf` |

## Next Steps

- Explore [Module 2: Digital Twin & Simulation](02-module-gazebo-unity.md) to build realistic physics
- Review [URDF/SDF Templates](../simulation/urdf/) in the project repository
- Run [Example Launches](../simulation/ros2_ws/) to see ROS 2 in action

---

**References**: See the References section for ROS 2 documentation and peer-reviewed papers on middleware architectures.

**Governance**: This module aligns with the Physical AI Constitution v1.0.0 principle of modular, testable design.
