Simulation scaffolding for Phase 2 — Gazebo & Unity

Purpose
- Provide reproducible simulation skeletons for humanoid robots used in Phase 2 of the hackathon.
- Simulation-first approach: validate algorithms in simulation before any real-world deployment.

Scope
- URDF and SDF templates for a basic humanoid description
- ROS 2 package skeleton (`robot_description`) with example launch file
- TDD-style reproducibility checks and instructions for RTX-enabled workstation or cloud GPU

Dependencies (recommended)
- Ubuntu 22.04 LTS (or compatible)
- ROS 2 Humble or Rolling (match project policy) with `ros-<distro>-desktop` installed
- Gazebo (citadel/ or compatible with ROS2 distro) or Ignition / Gazebo Fortress per your setup
- Python 3.10+
- Optional: NVIDIA GPU drivers + CUDA for Unity / Isaac GPU acceleration

Quick start (local dev)
1. Source ROS 2: `source /opt/ros/humble/setup.bash`
2. From `simulation/ros2_ws` run (after workspace build):

```bash
# build workspace (example)
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_description sim_launch.py
```

Reproducibility
- See `simulation/tests/reproducibility.md` for acceptance tests and reproducibility checks.

Notes
- URDF/SDF templates are intentionally minimal and should be extended for joint limits, inertial values, and realistic mass distribution before sim-to-real testing.
- Keep all modifications under `simulation/` and maintain versioning for sim parameter changes.
