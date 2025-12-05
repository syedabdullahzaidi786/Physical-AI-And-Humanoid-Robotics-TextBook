---
id: 005
title: Phase 2 Scaffolding — Gazebo & Unity Simulation Skeletons
stage: simulation
date: 2025-12-05
surface: agent
model: GPT-5 mini
feature: ai-classroom
branch: 001-ai-classroom-research
user: architect
command: /specify
labels: ["ai-classroom", "phase-2", "simulation", "gazebo", "ros2"]
links:
  spec: specs/ai-classroom/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - simulation/urdf/humanoid_template.urdf.xacro
  - simulation/sdf/humanoid_template.sdf
  - simulation/ros2_ws/src/robot_description/launch/sim_launch.py
  - simulation/tests/reproducibility.md
  - simulation/README.md
tests:
  - "TDD: URDF parses to valid URDF via xacro"
  - "TDD: ros2 launch runs robot_state_publisher and simulator"
  - "TDD: camera topic publishes after launch"
---

## Prompt

Create Phase 2 scaffolding: URDF/SDF templates, ROS 2 package skeleton, launch file, and reproducibility tests. Ensure instructions for RTX-enabled workstation and cloud runs.

## Summary

- Files created under `simulation/`:
  - `urdf/humanoid_template.urdf.xacro`
  - `sdf/humanoid_template.sdf`
  - `ros2_ws/src/robot_description/package.xml`
  - `ros2_ws/src/robot_description/launch/sim_launch.py`
  - `tests/reproducibility.md`
  - `README.md`

## Next Actions
1. Populate URDF with accurate inertial values & joint parameters for realistic physics
2. Add sensor plugins (LiDAR, IMU) and calibration parameters
3. Create Unity or NVIDIA Isaac scene templates for cross-simulator validation
4. Add CI job to run reproduciiblity smoke tests (GitHub Actions / Azure Pipelines)

---

Phase 2 scaffolding created. Ready for expansion and integration tests.
