# MEER Rover — Workspace README
# ================================

# autonomous_meer — NAGC 2026 ROS 2 Workspace

A production-grade, modular ROS 2 (Jazzy Jalisco) workspace for the **MEER Rover**
competing in the **NAGC 2026 Autonomous Navigation Mission**.

---

## Package Structure

```
autonomous_meer/
└── src/
    ├── meer_description/      # URDF/Xacro robot model + Gazebo plugins
    ├── meer_hardware/         # ros2_control SystemInterface (UART → ESP32)
    ├── meer_perception/       # Sensor processing, obstacle detection
    ├── meer_navigation/       # Nav2 integration, waypoint following
    ├── meer_mission_control/  # High-level FSM, competition sequencer
    └── meer_bringup/          # Top-level launch files + controller configs
```

---

## Quick Start (Simulation)

```bash
# 1. Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# 2. Build the workspace
cd ~/autonomous_meer
colcon build --symlink-install
source install/setup.bash

# 3. Launch simulation
ros2 launch meer_bringup sim.launch.py

# 4. Teleoperate (in a new terminal)
source ~/autonomous_meer/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/skid_steer_controller/cmd_vel_unstamped
```

---

## URDF Validation

```bash
# View the robot model without Gazebo
ros2 launch meer_description display.launch.py

# Validate xacro syntax
xacro src/meer_description/urdf/meer_rover.urdf.xacro use_sim:=true | \
  check_urdf /dev/stdin
```

---

## Package Details

### `meer_description`
- **`urdf/meer_rover.urdf.xacro`** — Top-level robot description.
  All inertial values sourced from SolidWorks URDF export.
  Dummy geometry (boxes/cylinders) — replace with `meshes/` STLs.
- **`urdf/ros2_control.xacro`** — Defines 4 driven wheels with
  `velocity` command and `position`/`velocity` state interfaces.
  Passive rockers excluded from `ros2_control`.
- **`urdf/gazebo_plugins.xacro`** — Loads `gz_ros2_control::GazeboSimROS2ControlPlugin`.

### `meer_hardware`
- Implements `hardware_interface::SystemInterface`.
- Hardware chain: `controller_manager → MeerSystemInterface → UART →
  ESP32 (micro-ROS) → Cytron MDDS30 → 24 V DC motors`.
- In simulation: `use_sim:=true` swaps to `gz_ros2_control/GazeboSimSystem`.
- See `include/meer_hardware/meer_system_interface.hpp` for full docs.

### `meer_bringup`
- **`launch/sim.launch.py`** — Full simulation bringup.
- **`config/controllers.yaml`** — `joint_state_broadcaster` +
  `skid_steer_drive_controller` configuration.

### `meer_navigation`
- Integrates Nav2 (`navigate_to_pose` action).
- **`config/nav2_params.yaml`** — Tuned for arena navigation.

### `meer_mission_control`
- Simple FSM: `IDLE → LOCALIZING → NAVIGATING → TASK_EXECUTING → RETURNING → DONE`.
- E-STOP via `/e_stop` topic.

---

## Controller Topic Map

| Topic | Direction | Type | Notes |
|-------|-----------|------|-------|
| `/skid_steer_controller/cmd_vel_unstamped` | ROS→Controller | `geometry_msgs/Twist` | Drive input |
| `/odom` | Controller→ROS | `nav_msgs/Odometry` | Wheel odometry |
| `/joint_states` | Broadcaster→ROS | `sensor_msgs/JointState` | All joints |
| `/clock` | Gazebo→ROS | `rosgraph_msgs/Clock` | Sim time |
| `/tf` | ROS→ROS | `tf2_msgs/TFMessage` | Full TF tree |

---

## Hardware Bring-up Checklist

- [ ] Flash micro-ROS firmware to ESP32
- [ ] Configure Cytron MDDS30 for Sign-Magnitude PWM mode
- [ ] Set `serial_port` in URDF `<hardware>` block
- [ ] Implement encoder readback in `meer_system_interface.cpp`
- [ ] Implement `write()` PWM duty + DIR packet transmission
- [ ] Tune `wheel_radius` and `wheel_separation_multiplier` in `controllers.yaml`
- [ ] Run `ros2 control list_hardware_interfaces` to verify interfaces load
- [ ] Calibrate odometry with `ros2 run meer_bringup odom_calib.py` (TBD)

---

## Dependencies

Install all ROS 2 binary dependencies:

```bash
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
```

Key packages:
- `ros-jazzy-ros2-control`
- `ros-jazzy-ros2-controllers`
- `ros-jazzy-gz-ros2-control`
- `ros-jazzy-ros-gz-sim`
- `ros-jazzy-ros-gz-bridge`
- `ros-jazzy-nav2-bringup`
- `ros-jazzy-robot-localization`

---

*Workspace scaffolded October 2026 — NAGC 2026 Team*
