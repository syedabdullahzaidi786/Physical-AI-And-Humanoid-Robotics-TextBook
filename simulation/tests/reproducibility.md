Simulation reproducibility & TDD checks (Phase 2)

Objectives
- Verify simulation runs reproducibly on local RTX-enabled workstation or cloud instance
- Define minimal acceptance tests for URDF/SDF, sensors, and ROS 2 integration

Environment
- Ubuntu 22.04 LTS
- ROS 2 Humble (or matching distro)
- Gazebo compatible with chosen ROS2 distro
- Python 3.10+

Acceptance Tests (TDD style)

1) Build & Launch Smoke Test
- Command:
  - `colcon build --symlink-install` (from `simulation/ros2_ws`)
  - `source install/setup.bash`
  - `ros2 launch robot_description sim_launch.py`
- Expected:
  - `robot_state_publisher` node starts
  - Gazebo (or compatible simulator) launches
  - No fatal errors in launch output

2) URDF Parse Test
- Command: `ros2 run xacro xacro simulation/urdf/humanoid_template.urdf.xacro > /tmp/humanoid.urdf`
- Expected: Valid URDF output and exit code 0

3) Sensor Topic Test
- After launch, verify camera topic present:
  - `ros2 topic list | grep camera` → returns a camera topic (e.g., `/head_camera/image_raw`)
- Expected: Topic exists and publishes messages at >0 Hz

4) Reproducibility Check
- Run smoke test 3 times and verify the simulation state (robot spawn successful) each time; capture logs
- Expected: No intermittent launch failures; basic sensor topics always appear

5) Hardware Acceleration Check (optional)
- If NVIDIA GPU present, verify GPU drivers and CUDA recognized by simulator (for Unity/Isaac integration)

Notes
- These tests are intentionally lightweight. For full sim-to-real validation extend tests to include physics parameter sweeps, sensor noise injection, and joint-limit verifications.
