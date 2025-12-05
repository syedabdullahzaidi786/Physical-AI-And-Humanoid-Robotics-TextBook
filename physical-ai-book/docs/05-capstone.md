---
id: capstone
title: "Module 5: Autonomous Humanoid Capstone Project"
sidebar_position: 5
---

# Module 5: Autonomous Humanoid Capstone Project

## Project Overview

The capstone project brings together all four modules (ROS 2, Gazebo/Unity, Isaac, VLA) to build a fully autonomous humanoid robot tutor for classroom deployment.

**Objectives:**
1. Design and simulate a humanoid robot morphology in Gazebo
2. Implement a perception stack using Isaac VSLAM
3. Train a control policy using reinforcement learning
4. Integrate conversational AI via VLA pipeline
5. Deploy safely in a simulated classroom environment
6. Measure educational ROI (learning gains, teacher workload reduction)

## Project Timeline

| Phase | Week | Deliverable |
|-------|------|------------|
| Design & Simulation | 1–2 | URDF/SDF, physics tuning, smoke tests |
| Perception Stack | 2–3 | VSLAM integration, camera calibration |
| Control Policy | 3–4 | RL training, behavior cloning from demos |
| Conversational AI | 4–5 | Speech→GPT→Actions pipeline, safety validation |
| Evaluation & ROI | 5–6 | Metrics collection, case study reporting |
| Deployment & Writeup | 6–7 | Final report, reproducibility archive |

## Part 1: Robot Design & Simulation

### Humanoid Morphology

A classroom humanoid robot requires:

**Degrees of Freedom (DOF):**
- Head: 2 DOF (pan, tilt) for attention
- Torso: 3 DOF (pitch, roll, yaw) for postural stability
- Arms: 3 DOF each (shoulder, elbow, wrist) for gesturing
- Hands: 1 DOF each (grip) for object manipulation
- Legs: 3 DOF each (hip, knee, ankle) for standing/walking
- **Total: ~20 DOF minimum**

### URDF Template

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="classroom_humanoid">
  <xacro:property name="base_mass" value="20"/>
  <xacro:property name="torso_mass" value="15"/>
  <xacro:property name="arm_mass" value="3"/>
  
  <link name="base_link">
    <inertial>
      <mass value="${base_mass}"/>
      <inertia ixx="1.0" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="1.0"/>
    </inertial>
  </link>
  
  <link name="torso_link">
    <inertial>
      <mass value="${torso_mass}"/>
      <inertia ixx="0.8" ixy="0" ixz="0" iyy="0.8" iyz="0" izz="0.8"/>
    </inertial>
  </link>
  
  <joint name="torso_joint" type="revolute">
    <parent link="base_link"/>
    <child link="torso_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>
</robot>
```

### Physics Tuning Checklist

- [ ] Robot stands stably without external support
- [ ] Joints move smoothly with no jerking
- [ ] Gravity behaves realistically (check inertial parameters)
- [ ] Collision detection works without sinking into ground
- [ ] Sensor simulation produces realistic noise (0.5–2% of measurement)

## Part 2: Perception Stack

### VSLAM Integration

1. **Calibrate Camera**: Obtain intrinsic parameters (focal length, principal point, distortion)
2. **Run Isaac VSLAM**: Initialize mapper and localizer
3. **Validate Odometry**: Compare ground-truth vs estimated pose

```python
from isaac_ros_vslam import VSLAM

vslam = VSLAM()
vslam.subscribe_image("/camera/image_raw")
vslam.subscribe_camera_info("/camera/camera_info")
vslam.publish_pose("/vslam/odometry")

# Verify map quality
while True:
    pose = vslam.get_pose()
    print(f"Robot pose: {pose}")
```

### Object Detection

Detect classroom objects (students, whiteboards, desks):

```python
import yolov8

detector = yolov8.YOLO("yolov8n.pt")  # Nano model for Jetson

detections = detector("/camera/image_raw")
for detection in detections:
    print(f"Detected: {detection.class_name} at {detection.bbox}")
```

## Part 3: Control Policy

### Behavior Cloning from Expert Demonstrations

Collect demonstrations of desired tutoring interactions:

```python
class BehaviorCapture:
    def __init__(self):
        self.trajectories = []
    
    def record_demo(self, demo_name):
        print(f"Recording demonstration: {demo_name}")
        states, actions = [], []
        
        # ... capture teleoperated robot actions ...
        
        self.trajectories.append((states, actions))
    
    def train_policy(self):
        model = torch.nn.Sequential(
            torch.nn.Linear(obs_dim, 256),
            torch.nn.ReLU(),
            torch.nn.Linear(256, action_dim)
        )
        optimizer = torch.optim.Adam(model.parameters())
        
        for epoch in range(100):
            for states, actions in self.trajectories:
                pred = model(states)
                loss = ((pred - actions) ** 2).mean()
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
        
        return model
```

### Safety Constraints

Enforce robot safety via constraint layers:

```python
class SafetyLayer:
    def __init__(self):
        self.max_speed = 0.5  # m/s
        self.max_joint_vel = 1.0  # rad/s
    
    def apply_safety(self, action):
        # Clip speeds
        action["linear_velocity"] = min(
            action["linear_velocity"], self.max_speed)
        
        # Check for obstacles
        if self.obstacle_detected():
            action["linear_velocity"] = 0
        
        return action
```

## Part 4: Conversational AI Integration

### Full VLA Pipeline

```python
class ClassroomRobotBrain:
    def __init__(self):
        self.speech = SpeechRecognizer()
        self.llm = IntentParser()
        self.perception = VSLAMNode()
        self.policy = PolicyNetwork()
        self.safety = SafetyLayer()
    
    def update(self):
        # Sense environment
        image, depth = self.perception.get_data()
        audio = self.microphone.get_audio()
        
        # Understand intent
        text = self.speech.transcribe(audio)
        intent = self.llm.parse(text)
        
        # Generate action
        policy_input = {
            "visual": image,
            "depth": depth,
            "intent": intent
        }
        action = self.policy(policy_input)
        
        # Apply safety
        safe_action = self.safety.apply_safety(action)
        
        # Execute
        self.robot.execute(safe_action)
```

## Part 5: Evaluation Metrics

### Learning Outcomes

Measure using **pre/post standardized tests**:
- **Cohen's d effect size** (target: 0.3–0.6 for moderate improvement)
- **Learning gain** = (post – pre) / (max – pre)
- **Engagement** = time-on-task / session-duration

### Teacher Workload

Track time savings:
- **Preparation time**: minutes spent creating lesson materials
- **Intervention time**: minutes spent on 1-on-1 help
- **Documentation**: minutes spent logging student progress

### System Metrics

Ensure robot reliability:
- **Uptime**: % of time system is operational
- **Task success rate**: % of attempted tasks completed successfully
- **Response latency**: time from student input to robot response (target: under 8 seconds)

## Part 6: Safety & Compliance Checklist

Before live trials in a classroom:

- [ ] ISO/IEC 13482 safety standards reviewed
- [ ] Emergency stop button tested and documented
- [ ] Parental consent forms obtained
- [ ] FERPA compliance verified (no data stored/shared without permission)
- [ ] ADA accessibility features tested
- [ ] Classroom environment safety reviewed (no obstacles, emergency exits clear)
- [ ] IRB approval obtained (if applicable for research)

## Part 7: Deployment & Publication

### Build Reproducibility Archive

```bash
tar -czf capstone_archive.tar.gz \
  simulation/urdf/ \
  simulation/sdf/ \
  simulation/ros2_ws/ \
  trained_models/ \
  research/ai-classroom/ \
  docs/
```

### Final Report Structure

1. **Executive Summary** (300 words) — Key findings for administrators
2. **Introduction** — Context and motivation
3. **Methods** — System design, training procedure, evaluation metrics
4. **Results** — Learning gains, workload reduction, system reliability
5. **Discussion** — Implications, limitations, future work
6. **References** — APA-formatted citations
7. **Appendices** — Code snippets, URDF templates, full case studies

### Publishing Venues

- **Venues for AI + Education**: ICER, FIE, SIGCSE
- **Venues for Robotics**: ICRA, IROS, RSS
- **Venues for HCI + Learning**: CHI, TEI, IDC

## Submission Checklist

- [ ] Code is well-documented and reproducible
- [ ] All claims are traceable to peer-reviewed sources
- [ ] Simulation runs successfully locally and in CI
- [ ] Report word count within guidelines (3,500–5,000)
- [ ] Figures and tables are publication-quality
- [ ] References are complete and formatted consistently
- [ ] PHR entries document all decisions and reviewers' feedback

## Next Steps

1. Choose a specific educational application (tutoring, behavioral support, etc.)
2. Begin Part 1 (Robot Design) and complete physics tuning
3. Move to Part 2 and calibrate the perception stack
4. Integrate Parts 3–4 incrementally and run smoke tests
5. Collect data for Part 5 and analyze metrics
6. Document findings in a publication-ready report

---

**Resources:**
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Tutorials](https://gazebosim.org/docs/)
- [Isaac Documentation](https://docs.nvidia.com/isaac/)

**Governance**: This capstone aligns with all principles of the Physical AI Constitution v1.0.0. Document decisions in PHR entries as you progress.
