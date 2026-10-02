# 🚀 MEER Rover — Complete Architecture Guide
### *Everything you need to know, from zero to autonomous rover*

> **Who is this for?** → New team members, collaborators, judges, or anyone who opens this folder and asks *"What IS this?!"*

---

## 🎯 THE BIG PICTURE — What Are We Building?

Imagine you're designing a **self-driving car** — but instead of driving on roads, it navigates a competition arena **completely on its own**, avoiding obstacles, reaching GPS waypoints, and completing tasks. That's exactly what **MEER Rover** is.

**MEER Rover** is a **6-wheeled autonomous ground robot** built for the **NAGC 2026 Autonomous Navigation Mission** — a prestigious robotics competition where rovers must navigate unknown terrain without any human control.

```
🏆 GOAL: Build a rover that can:
   ✅ Drive autonomously through an arena
   ✅ Avoid obstacles using sensors
   ✅ Navigate to GPS waypoints
   ✅ Execute competition tasks
   ✅ Return home safely — all by itself!
```

---

## 🌍 REAL-LIFE ANALOGY — Think of It This Way

| What we're building | Real-life equivalent |
|---|---|
| **MEER Rover** | A self-driving delivery robot |
| **ROS 2** (our framework) | The "operating system" of the robot — like Android for phones |
| **URDF model** | A digital blueprint / 3D model of the rover body |
| **Gazebo simulator** | A video game world to test the rover safely before real deployment |
| **Nav2 navigation stack** | Google Maps + self-driving AI, built for robots |
| **ESP32 microcontroller** | The "hands" that actually spin the motors |
| **ros2_control** | The translator between the robot brain and its hands |
| **UART communication** | The USB cable between laptop (brain) and Arduino (hands) |

---

## 🗂️ WORKSPACE FOLDER STRUCTURE — The Full Map

```
autonomous_meer/              ← ROOT WORKSPACE (like a project folder)
│
├── 📁 src/                   ← ALL SOURCE CODE lives here (you write here!)
│   │
│   ├── 📦 meer_description/  ← "What does the rover LOOK like?"
│   ├── 📦 meer_hardware/     ← "How does ROS talk to the MOTORS?"
│   ├── 📦 meer_perception/   ← "What does the rover SEE?"
│   ├── 📦 meer_navigation/   ← "How does the rover MOVE to a target?"
│   ├── 📦 meer_mission_control/ ← "What DECISIONS does the rover make?"
│   └── 📦 meer_bringup/      ← "How do we START everything together?"
│
├── 📁 build/                 ← AUTO-GENERATED (don't edit! colcon puts compiled files here)
├── 📁 install/               ← AUTO-GENERATED (final executables go here)
├── 📁 log/                   ← AUTO-GENERATED (build & run logs)
└── 📄 README.md              ← Quick-start instructions
```

> [!NOTE]
> **You only ever edit files inside `src/`**. The `build/`, `install/`, and `log/` folders are automatically generated when you run `colcon build`. Think of them like compiled `.class` files in Java — generated, not hand-written.

---

## 📦 THE 6 PACKAGES — Deep Dive

Each "package" in ROS 2 is like a **microservice** — it does one job and communicates with others through messages. Here's each one explained:

---

### 📦 1. `meer_description` — The Robot's Blueprint

> **Analogy:** *Like an architect's blueprint + 3D model of a building, but for a robot.*

**What it does:** Defines the physical shape, dimensions, joints, and sensors of the MEER rover in a format that both the simulator and the control system can understand.

```
meer_description/
├── urdf/
│   ├── meer_rover.urdf.xacro     ← Main robot body: links, joints, wheels
│   ├── ros2_control.xacro        ← Which joints are motorized & how
│   ├── gazebo_plugins.xacro      ← Connects Gazebo simulator to ROS 2
│   ├── inertia_macros.xacro      ← Physics math (mass, moment of inertia)
│   └── materials.xacro           ← Colors (black wheels, orange rockers, etc.)
├── config/
│   └── meer_rover.rviz           ← RViz visualization settings
├── launch/
│   └── display.launch.py         ← Launch: "show me the robot model"
└── meshes/                       ← (Future) 3D STL files for realistic visuals
```

**Key Concepts Explained:**

| Term | Simple Explanation |
|---|---|
| **URDF** | Unified Robot Description Format — XML file describing robot shape |
| **Xacro** | Like HTML templates for URDF — allows variables and macros |
| **Link** | A rigid body part (e.g., `base_link`, `FLW` = Front Left Wheel) |
| **Joint** | Connection between two links (can be fixed, rotating, sliding) |
| **Rocker** | The suspension arm that pivots — like a see-saw connecting two wheels |
| **Inertia** | How hard it is to rotate a body — needed for realistic physics simulation |

**The Rover's Mechanical Structure:**

```
                    base_link (main body)
                   /                    \
        left_bearing (fixed)      right_bearing (fixed)
               |                              |
        Left_rocker (PASSIVE pivot)    right_rocker (PASSIVE pivot)
          /          \                   /           \
       FLW          BLW              FRW            BRW
  (Front Left)  (Back Left)    (Front Right)   (Back Right)
  [DRIVEN]      [DRIVEN]       [DRIVEN]        [DRIVEN]
```

> [!TIP]
> The **rocker suspension** is genius! When the rover goes over a bump, the rocker pivots passively, keeping all 4 driven wheels on the ground — just like Mars rovers. The rockers are **not motorized** — they're passive physics.

---

### 📦 2. `meer_hardware` — The Brain-to-Motor Bridge

> **Analogy:** *Like a USB driver on your computer — it translates high-level commands ("move at 2 m/s") into the low-level electrical signals the motors understand.*

**What it does:** Implements the `ros2_control` hardware interface so the navigation stack can command wheel velocities, and the physical motors (via ESP32) actually obey.

```
meer_hardware/
├── include/meer_hardware/
│   └── meer_system_interface.hpp  ← Class definition & documentation
├── src/
│   └── meer_system_interface.cpp  ← Implementation (read/write to ESP32)
├── meer_hardware_plugin.xml       ← Tells ROS 2 this is a pluginlib plugin
└── package.xml / CMakeLists.txt   ← Build configuration
```

**The Communication Chain:**

```
┌─────────────────────────────────────────────────────────────────┐
│                     ROS 2 WORLD (Linux PC)                      │
│                                                                 │
│  Nav2 → "turn left at 1.5 rad/s"                               │
│           ↓                                                     │
│  skid_steer_controller → wheel velocities [FL, BL, FR, BR]     │
│           ↓                                                     │
│  MeerSystemInterface::write()  ← YOU ARE HERE                  │
│           ↓  UART serial port (/dev/ttyUSB0 @ 921600 baud)     │
└─────────────────────────────────────────────────────────────────┘
           ↓  8-byte packet: [STX][FL][BL][FR][BR][DIR][CHK][ETX]
┌─────────────────────────────────────────────────────────────────┐
│                  ESP32 MICROCONTROLLER                          │
│                                                                 │
│  micro-ROS → decode packet → PWM + DIR signals                 │
│           ↓                                                     │
│  4× Cytron MDDS30 Motor Drivers (Sign-Magnitude PWM mode)      │
│           ↓                                                     │
│  4× 24V DC Motors spin!  🎡                                    │
└─────────────────────────────────────────────────────────────────┘
```

**Lifecycle of the Hardware Interface:**

```
on_init()       → Validate URDF config, read serial port name
    ↓
on_configure()  → Open UART connection to ESP32
    ↓
on_activate()   → Start motors (send zero PWM initially)
    ↓
read()  ←──┐   → Read encoder data from ESP32 (wheel position/velocity)
write() ───┘   → Send velocity commands as PWM+DIR to ESP32
    ↓
on_deactivate() → Send STOP command (safety!) before shutdown
```

> [!IMPORTANT]
> **Current Status:** The `read()` and `write()` methods have **TODO stubs** — they simulate perfect motors now. The actual UART packet transmission to the ESP32 still needs to be implemented. See the Hardware Bring-up Checklist.

---

### 📦 3. `meer_perception` — The Rover's Eyes

> **Analogy:** *Like the eyes + brain of a car's collision avoidance system — it watches sensor data and shouts "OBSTACLE AHEAD!" when needed.*

**What it does:** Processes raw sensor data (LiDAR, cameras) and publishes meaningful information like "there's an obstacle 0.3 meters ahead."

```
meer_perception/
├── src/
│   └── obstacle_detector.cpp     ← LiDAR scan processor
├── launch/
│   └── perception.launch.py      ← Starts perception nodes
└── config/                       ← Sensor tuning parameters
```

**How It Works:**

```
LiDAR Sensor
    ↓ /lidar/scan  (360° distance measurements)
ObstacleDetector Node
    ↓ checks: is any reading < 0.5 meters?
    ├── /obstacle_detected  [Bool]         → True/False flag
    └── /nearest_obstacle   [PointStamped] → Where is the closest object?
```

**Published Topics:**
- `/obstacle_detected` → `True` if something is within `min_obstacle_dist` meters
- `/nearest_obstacle` → Exact position of the closest detected object

> [!NOTE]
> **Current Status:** Basic min-range detection is implemented. Future plans include direction-specific detection (only front ±30°), PointCloud2 support for depth cameras, and integration with the mission state machine.

---

### 📦 4. `meer_navigation` — The Rover's GPS & Pathfinder

> **Analogy:** *Like Google Maps + a self-driving AI for the rover. You give it a destination; it figures out the safest, fastest path and drives there.*

**What it does:** Uses the **Nav2 navigation stack** (ROS 2's standard autonomous navigation framework) to plan paths and drive the rover to target waypoints.

```
meer_navigation/
├── src/
│   └── waypoint_navigator.cpp    ← Sends goals to Nav2 action server
├── config/
│   └── nav2_params.yaml          ← All Nav2 tuning parameters
└── launch/
    └── navigation.launch.py      ← Starts Nav2 stack
```

**The Navigation Pipeline:**

```
Mission Control says: "Go to waypoint (x=5.0, y=3.2)"
    ↓
WaypointNavigator → NavigateToPose action goal
    ↓
┌─────────────── NAV2 STACK ───────────────────────┐
│                                                  │
│  planner_server     → Find a path (A* algorithm) │
│       ↓ global plan                              │
│  controller_server  → Follow the path            │
│  (RegulatedPurePursuitController)                │
│       ↓ /cmd_vel (linear + angular velocity)     │
│  velocity_smoother  → Smooth out jerky commands  │
│       ↓                                          │
│  skid_steer_controller → Individual wheel speeds │
└──────────────────────────────────────────────────┘
    ↓
Motors spin → Rover moves!
```

**Key Nav2 Components Configured:**

| Component | What it does | Our setting |
|---|---|---|
| **NavFn Planner** | Plans global path (like Google Maps route) | Dijkstra's algorithm |
| **RegulatedPurePursuit** | Local path following controller | Max speed: 0.5 m/s |
| **Local Costmap** | 3m × 3m grid of "danger zones" around rover | Updates at 5 Hz |
| **Global Costmap** | Full arena map with obstacles | Updates at 1 Hz |
| **Behavior Server** | Recovery behaviors (spin in place, back up) | Enabled |
| **Velocity Smoother** | Prevents jerky velocity changes | Max accel: 2.5 m/s² |

---

### 📦 5. `meer_mission_control` — The Rover's Brain / Decision Maker

> **Analogy:** *Like a military mission commander — it knows the overall plan, gives orders to each department, and reacts to emergencies.*

**What it does:** Runs a **Finite State Machine (FSM)** that orchestrates the entire mission — from startup to task completion to returning home.

```
meer_mission_control/
├── src/
│   └── mission_control_node.cpp  ← The FSM brain
└── launch/
    └── mission.launch.py         ← Starts the mission
```

**The Mission State Machine:**

```
         ┌─────────────────────────────────────────────────────┐
         │                  MISSION FSM                        │
         └─────────────────────────────────────────────────────┘

  START
    │
    ▼
 ┌──────┐     auto_start=true      ┌────────────┐
 │ IDLE │ ─────────────────────── ▶│ LOCALIZING │
 └──────┘                          └────────────┘
                                         │ GPS/AMCL lock confirmed
                                         ▼
                                   ┌───────────┐
                    ┌──────────────│ NAVIGATING│
                    │              └───────────┘
                    │                    │ arrived at task zone
                    │                    ▼
                    │             ┌───────────────┐
                    │             │TASK_EXECUTING │
                    │             └───────────────┘
                    │                    │ task complete
                    │                    ▼
                    │              ┌──────────┐
                    │              │RETURNING │
                    │              └──────────┘
                    │                    │ back at start
                    │                    ▼
                    │               ┌──────┐
                    │               │ DONE │
                    │               └──────┘
                    │
  /e_stop received ─┼──────────────────────────────────▶ ┌───────┐
  (any state)       │                                     │ FAULT │
                    │                                     └───────┘
                    └── (on any navigation failure)
```

**Communication:**
- **Publishes:** `/mission_status` (String) — current state name
- **Subscribes:** `/e_stop` (String) — emergency stop trigger from any source

> [!WARNING]
> The **E-STOP** (`/e_stop` topic) immediately halts the mission and enters `FAULT` state. This is a **safety-critical** feature — always ensure this works before running in real hardware.

---

### 📦 6. `meer_bringup` — The Mission Control Room

> **Analogy:** *Like a stage director's cue sheet — it knows exactly what to start, in what order, and with what settings to put on the whole show.*

**What it does:** Contains the master launch files and configuration that starts ALL packages together in the right order.

```
meer_bringup/
├── launch/
│   └── sim.launch.py      ← "Start everything for simulation"
└── config/
    └── controllers.yaml   ← Drive controller configuration
```

**What `sim.launch.py` starts:**
1. Gazebo simulation world
2. Robot state publisher (publishes TF transforms from URDF)
3. `ros2_control` node with Gazebo plugin (simulated motors)
4. `joint_state_broadcaster` (reports all joint positions)
5. `skid_steer_drive_controller` (converts cmd_vel → wheel velocities)

**`controllers.yaml` key settings:**
```yaml
skid_steer_drive_controller:
  wheel_separation: 0.43 m  ← Distance between left and right wheels
  wheel_radius: 0.10 m      ← Wheel radius (10 cm)
  # Controls 4 wheels: front_left, back_left, front_right, back_right
```

---

## 🔄 HOW EVERYTHING CONNECTS — The Data Flow

This is the complete picture of how data flows through the system:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    MEER ROVER — SYSTEM DATA FLOW                        │
└─────────────────────────────────────────────────────────────────────────┘

  SENSORS                PERCEPTION              NAVIGATION
  ────────               ──────────              ──────────
  LiDAR ──────────────▶ ObstacleDetector ──────▶ Nav2 Costmaps
  /lidar/scan            /obstacle_detected       (danger zones)
                         /nearest_obstacle
                                                       │
  GPS/IMU ─────────────────────────────────────▶ Localization
  (future)                                       (AMCL / EKF)
                                                       │
                                                       ▼
  MISSION CONTROL                              PLANNING
  ───────────────                              ────────
  MissionControlNode ──── "go to pose" ──────▶ Nav2 Planner
  (FSM: IDLE→DONE)         action goal          (A* path)
         │                                          │
         │ /mission_status                          ▼
         │                                   CONTROL
         ▼                                   ───────
  /e_stop ──── FAULT state             Nav2 Controller
                                       (RegPurePursuit)
                                             │
                                             ▼ /cmd_vel
                                   ACTUATION
                                   ─────────
                              skid_steer_controller
                              (wheel velocities)
                                             │
                                             ▼
                              MeerSystemInterface
                              (ros2_control hardware)
                                             │ UART
                                             ▼
                              ESP32 + Cytron MDDS30
                              (PWM → 24V DC motors)
                                             │
                                             ▼
                                     🚗 ROVER MOVES!
```

---

## 📡 THE TOPIC MAP — How Nodes Talk to Each Other

In ROS 2, nodes communicate by **publishing** and **subscribing** to **topics** (like WhatsApp group chats — one person posts, everyone who joined that chat receives it).

```
Topic Name                          Type                    Who Sends → Who Receives
─────────────────────────────────────────────────────────────────────────────────────
/cmd_vel                            geometry_msgs/Twist     Nav2 → skid_steer_controller
/skid_steer_controller/             geometry_msgs/Twist     Teleop keyboard → controller
  cmd_vel_unstamped
/odom                               nav_msgs/Odometry       controller → Nav2 localization
/joint_states                       sensor_msgs/JointState  broadcaster → RViz / Nav2
/clock                              rosgraph_msgs/Clock     Gazebo → all nodes
/tf & /tf_static                    tf2_msgs/TFMessage      robot_state_pub → all
/lidar/scan                         sensor_msgs/LaserScan   LiDAR → ObstacleDetector, Nav2
/obstacle_detected                  std_msgs/Bool           ObstacleDetector → MissionCtrl
/mission_status                     std_msgs/String         MissionCtrl → monitoring
/e_stop                             std_msgs/String         Any source → MissionCtrl
```

---

## 🔧 BUILD SYSTEM — How It All Gets Compiled

```
You write code in src/
        │
        ▼
  colcon build         ← The ROS 2 build tool (like Maven for Java)
        │
        ├── reads CMakeLists.txt in each package
        ├── compiles C++ with g++
        ├── generates Python wrappers
        └── puts results in:
              build/    ← intermediate files
              install/  ← final executables & libraries

        ▼
source install/setup.bash   ← "Activate" the workspace (like a Python venv)
        │
        ▼
ros2 launch meer_bringup sim.launch.py   ← 🚀 Everything starts!
```

**Each package has these standard files:**
- `CMakeLists.txt` — Build instructions (like a Makefile)
- `package.xml` — Package metadata + dependencies (like `package.json` in Node.js)

---

## 🎯 CURRENT STATUS & FUTURE ROADMAP

### ✅ What's Done (Phase 1 — Foundation)

```
[██████████] 100%  Rover URDF model (6-wheel rocker suspension)
[██████████] 100%  ros2_control hardware interface skeleton
[██████████] 100%  Gazebo simulation integration
[██████████] 100%  Skid-steer drive controller
[██████████] 100%  Nav2 configuration (params tuned for arena)
[██████████] 100%  Mission FSM skeleton
[██████████] 100%  Basic obstacle detector
[██████████] 100%  Waypoint navigator boilerplate
```

### 🔨 In Progress / TODO (Phase 2 — Real Hardware)

```
[██░░░░░░░░]  20%  UART/micro-ROS ESP32 communication (write stubs exist)
[█░░░░░░░░░]  10%  Encoder readback from ESP32
[░░░░░░░░░░]   0%  GPS waypoint loading from YAML
[░░░░░░░░░░]   0%  GPS → UTM → map frame conversion
[░░░░░░░░░░]   0%  3D mesh STL files for realistic visuals
[░░░░░░░░░░]   0%  Odometry calibration script
[░░░░░░░░░░]   0%  AMCL/EKF localization tuning
```

### 🚀 Future Plans (Phase 3 — Competition Ready)

```
[ ] Add camera + ArUco marker detection for task execution
[ ] Implement gate traversal behavior
[ ] Add artefact pickup arm control
[ ] Full arena map pre-loading
[ ] Competition run automation
[ ] Real-world testing & tuning
```

---

## 🖥️ HOW TO RUN IT — Step by Step

### Simulation (No Physical Robot Needed!)

```bash
# Step 1: Activate ROS 2
source /opt/ros/jazzy/setup.bash

# Step 2: Go to workspace and build
cd ~/autonomous_meer
colcon build --symlink-install

# Step 3: Activate your built packages
source install/setup.bash

# Step 4: Launch the full simulation!
ros2 launch meer_bringup sim.launch.py
# → Gazebo opens with the rover in an empty world
# → All controllers start automatically

# Step 5 (new terminal): Drive it manually to test!
source ~/autonomous_meer/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/skid_steer_controller/cmd_vel_unstamped
# → Use WASD keys to drive!
```

### Just View the Robot Model

```bash
ros2 launch meer_description display.launch.py
# → Opens RViz with the 3D rover model
# → Great for checking URDF changes
```

### Validate URDF Syntax

```bash
xacro src/meer_description/urdf/meer_rover.urdf.xacro use_sim:=true | check_urdf /dev/stdin
```

---

## 🔑 KEY TERMS GLOSSARY

| Term | Plain English |
|---|---|
| **ROS 2** | Robot Operating System 2 — middleware framework for robots. Like an OS for robot apps. |
| **Node** | A single running program that does one job (e.g., obstacle_detector) |
| **Topic** | A named data stream. Nodes publish to it; others subscribe. Like a group chat. |
| **Action** | Like an RPC call with progress updates. Used for long tasks (e.g., navigate to pose). |
| **URDF** | The robot's body blueprint in XML format |
| **Xacro** | Macro system for URDF — adds variables, loops, includes |
| **Gazebo** | 3D physics simulator. Lets you test the rover without real hardware. |
| **Nav2** | Navigation2 — ROS 2's standard autonomous navigation stack |
| **ros2_control** | Framework for real-time robot control (motors, arms, grippers) |
| **SystemInterface** | The plugin that connects ros2_control to actual hardware |
| **ESP32** | A cheap microcontroller that runs the motor driver code |
| **micro-ROS** | ROS 2 running on tiny microcontrollers like ESP32 |
| **Cytron MDDS30** | Motor driver that converts digital signals to motor power |
| **UART** | Serial communication protocol (like USB but simpler) |
| **PWM** | Pulse Width Modulation — controls motor speed by rapidly switching power on/off |
| **Skid-Steer** | Drive style where turning is done by making left/right wheels go different speeds (like tanks) |
| **FSM** | Finite State Machine — a system that's always in one state and transitions based on events |
| **Odometry** | Estimating position by tracking wheel rotations (like counting steps to estimate distance walked) |
| **Costmap** | A grid map where each cell has a "danger score" — Nav2 avoids high-cost areas |
| **TF / Transform** | Coordinate system relationships. "Where is the LiDAR relative to the wheel?" |
| **Colcon** | ROS 2's build tool — compiles all packages in the workspace |
| **AMCL** | Adaptive Monte Carlo Localization — figuring out "where am I?" on a map |
| **EKF** | Extended Kalman Filter — fuses GPS + IMU + odometry for better position estimation |
| **NAGC 2026** | National Autonomous Ground Challenge 2026 — the competition we're entering |

---

## 🏗️ ARCHITECTURE DIAGRAM

```
╔════════════════════════════════════════════════════════════════════╗
║                    MEER ROVER ARCHITECTURE                         ║
╠════════════════════════════════════════════════════════════════════╣
║                                                                    ║
║  ┌──────────────────────────────────────────────────────────────┐  ║
║  │                   HIGH-LEVEL (Decision)                      │  ║
║  │                                                              │  ║
║  │    meer_mission_control  →  Mission FSM  →  /mission_status  │  ║
║  └─────────────────────────────┬────────────────────────────────┘  ║
║                                │ navigate_to_pose actions          ║
║  ┌─────────────────────────────▼────────────────────────────────┐  ║
║  │                   MID-LEVEL (Navigation)                     │  ║
║  │                                                              │  ║
║  │  meer_navigation  →  Nav2 Stack  →  WaypointNavigator        │  ║
║  │     ↑ LiDAR costmaps from meer_perception                    │  ║
║  └─────────────────────────────┬────────────────────────────────┘  ║
║                                │ /cmd_vel                          ║
║  ┌─────────────────────────────▼────────────────────────────────┐  ║
║  │                   LOW-LEVEL (Control)                        │  ║
║  │                                                              │  ║
║  │  meer_bringup  →  ros2_control  →  skid_steer_controller     │  ║
║  │     meer_description (URDF) defines all joints               │  ║
║  └─────────────────────────────┬────────────────────────────────┘  ║
║                                │ UART                              ║
║  ┌─────────────────────────────▼────────────────────────────────┐  ║
║  │                   HARDWARE (Physical)                        │  ║
║  │                                                              │  ║
║  │  meer_hardware  →  ESP32  →  Cytron MDDS30  →  DC Motors    │  ║
║  └──────────────────────────────────────────────────────────────┘  ║
║                                                                    ║
╚════════════════════════════════════════════════════════════════════╝
```

---

## 📋 HARDWARE BRING-UP CHECKLIST

Before running on the **real physical rover** (not simulation), complete these steps:

- [ ] Flash micro-ROS firmware to ESP32
- [ ] Wire Cytron MDDS30 in Sign-Magnitude PWM mode (DIR + PWM pins per channel)
- [ ] Set correct `serial_port` in URDF `<hardware>` block (e.g., `/dev/ttyUSB0`)
- [ ] Implement encoder readback in `meer_system_interface.cpp`
- [ ] Implement `write()` — convert velocity → PWM duty + DIR packet
- [ ] Tune `wheel_radius` and `wheel_separation_multiplier` in `controllers.yaml`
- [ ] Verify: `ros2 control list_hardware_interfaces`
- [ ] Calibrate odometry: `ros2 run meer_bringup odom_calib.py` *(TBD)*

---

## 📦 INSTALLING DEPENDENCIES

```bash
# Update package list
sudo apt update

# Install ALL ROS 2 dependencies automatically
rosdep install --from-paths src --ignore-src -r -y

# Key packages installed:
#   ros-jazzy-ros2-control
#   ros-jazzy-ros2-controllers
#   ros-jazzy-gz-ros2-control
#   ros-jazzy-ros-gz-sim
#   ros-jazzy-ros-gz-bridge
#   ros-jazzy-nav2-bringup
#   ros-jazzy-robot-localization
```

---

*📅 Workspace created: October 2026 | 🏆 Competition: NAGC 2026 | 🤖 Robot: MEER Rover*

*🛠️ Framework: ROS 2 Jazzy Jalisco | 🎮 Simulator: Gazebo Harmonic | 🧠 Navigation: Nav2*
