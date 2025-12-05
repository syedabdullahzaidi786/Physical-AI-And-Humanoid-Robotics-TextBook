---
id: module-gazebo-unity
title: "Module 2: Digital Twin & Simulation (Gazebo & Unity)"
sidebar_position: 2
---

# Module 2: Digital Twin & Simulation (Gazebo & Unity)

## What is a Digital Twin?

A **digital twin** is a virtual replica of a physical system that mirrors real-world behavior in simulation. For robotics education:

- **Risk-Free Testing**: Experiment with algorithms before physical deployment
- **Rapid Prototyping**: Iterate on designs quickly without hardware costs
- **Reproducibility**: Exact replay of scenarios for validation and debugging
- **Scalability**: Run parallel simulations for parameter sweeps

## Gazebo: Open-Source Physics Simulation

Gazebo is the standard simulation environment for ROS 2 robots. Key features:

- **Physics Engines**: ODE, Bullet, DART for accurate dynamics
- **Sensor Simulation**: Camera, LiDAR, IMU, depth sensors with realistic noise
- **ROS 2 Integration**: Native topic publishing and service calls
- **Plugins**: Extensible architecture for custom sensors and actuators

### URDF & SDF: Robot Description Formats

**URDF** (Unified Robot Description Format):
- Human-readable XML
- Describes kinematic structure and basic properties
- Lacks physics detail; relies on Gazebo for physics parameters

```xml
<robot name="humanoid">
  <link name="base_link">
    <inertial>
      <mass value="20"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry><box size="0.3 0.2 0.1"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.3 0.2 0.1"/></geometry>
    </collision>
  </link>
  <joint name="right_hip_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_leg"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2"/>
  </joint>
</robot>
```

**SDF** (Simulation Description Format):
- Complete physics specification for Gazebo
- Includes friction, contact properties, and plugin definitions

```xml
<sdf version="1.6">
  <world name="default">
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <model name="humanoid">
      <link name="base_link">
        <inertial>
          <mass>20</mass>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### Sensor Simulation: Camera Plugin

```xml
<plugin name="camera_controller" filename="libgazebo_ros_camera.so">
  <ros>
    <remapping>~/image_raw:=/camera/image_raw</remapping>
    <remapping>~/camera_info:=/camera/camera_info</remapping>
  </ros>
  <camera_name>camera</camera_name>
  <image_width>640</image_width>
  <image_height>480</image_height>
  <camera_info_topic>camera_info</camera_info_topic>
</plugin>
```

## Unity for Advanced Visualization

**Unity** is popular for creating photorealistic simulations and interactive demos:

- **Graphics**: High-fidelity rendering for curriculum materials
- **VR/AR Integration**: Immersive learning environments
- **Cross-Platform**: Deploy to web, mobile, desktop

### Connecting Unity to ROS 2

Use **ROS 2 For Unity** bridge:

```csharp
using ROS2;
using geometry_msgs.msg;

public class RobotController : MonoBehaviour {
    private IPublisher<Twist> cmdVelPub;
    
    void Start() {
        ROS2.Init();
        var node = ROS2.CreateNode("robot_controller");
        cmdVelPub = node.CreatePublisher<Twist>("/cmd_vel");
    }
    
    void Update() {
        var msg = new Twist();
        msg.Linear.X = Input.GetAxis("Vertical");
        msg.Angular.Z = Input.GetAxis("Horizontal");
        cmdVelPub.Publish(msg);
    }
}
```

## Creating a Simulation Scenario

### Step 1: Define Robot URDF

Create `humanoid_template.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">
  <!-- Define macro for link with inertia -->
  <xacro:macro name="link_with_inertia" params="name mass">
    <link name="${name}">
      <inertial>
        <mass value="${mass}"/>
        <inertia ixx="${mass*0.01}" ixy="0" ixz="0" 
                 iyy="${mass*0.01}" iyz="0" izz="${mass*0.01}"/>
      </inertial>
    </link>
  </xacro:macro>

  <xacro:link_with_inertia name="base_link" mass="20"/>
  <xacro:link_with_inertia name="torso" mass="15"/>
  
  <joint name="torso_joint" type="revolute">
    <parent link="base_link"/>
    <child link="torso"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="2"/>
  </joint>
</robot>
```

### Step 2: Launch in Gazebo

Create `sim_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('robot_description').find('robot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'humanoid_template.urdf')
    
    with open(urdf_file) as f:
        robot_urdf = f.read()
    
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='gazebo',
            arguments=['-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so']
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'humanoid', '-topic', 'robot_description']
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_urdf}]
        ),
    ])
```

### Step 3: Run Simulation

```bash
ros2 launch robot_description sim_launch.py
```

## Physics Tuning

Key parameters affect simulation accuracy and speed:

| Parameter | Impact | Typical Value |
|-----------|--------|---------------|
| `max_step_size` | Physics time-step | 0.001 (1 ms) |
| `real_time_factor` | Speed vs accuracy | 1.0 (real-time) |
| `gravity` | Environmental force | [0, 0, -9.81] m/s² |
| `friction` | Surface interaction | 1.0 (default) |

Reduce `max_step_size` for accuracy; increase `real_time_factor` for faster runs (at the cost of fidelity).

## Sensor Noise & Realism

Add realistic sensor noise to improve sim-to-real transfer:

```xml
<noise>
  <type>gaussian</type>
  <mean>0</mean>
  <stddev>0.01</stddev>
</noise>
```

## Best Practices

1. **Validate in Sim First**: Always verify behavior in simulation before real deployment
2. **Incremental Complexity**: Start with simple shapes; add visual meshes later
3. **Performance**: Use simple collision geometries; cache expensive computations
4. **Reproducibility**: Record and replay simulation runs for debugging
5. **Sensor Fidelity**: Tune noise and update rates to match real sensors

## Troubleshooting Gazebo

| Issue | Cause | Solution |
|-------|-------|----------|
| Robot sinks into ground | Collision issues | Check inertia and contact parameters |
| Slow simulation | Physics complexity | Reduce polygon count, simplify geometry |
| Topics not publishing | Plugin not loaded | Verify SDF plugin paths and ROS 2 namespace |

## Next Steps

- Advance to [Module 3: AI-Robot Brain](03-module-isaac.md) for perception and learning
- Review [URDF Templates](https://github.com/physical-ai-project/simulation/tree/main/urdf) in the project repository
- Run reproducibility tests

---

**References**: See the References section for Gazebo documentation and physics simulation papers.

**Governance**: Simulation-first development is a core principle of the Physical AI Constitution v1.0.0.
