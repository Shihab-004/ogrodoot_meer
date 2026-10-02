# 🏆 NAGC 2026 — Autonomous Navigation Mission Specification

**Official Mission Rules & Technical Architecture Guide for MEER Rover Autonomous Team**

---

## 📋 1. Mission Overview

* **Mission Name:** 8. Autonomous Navigation Mission
* **Total Marks:** 100 Marks (+ Time Completion Bonus)
* **Total Time Limit:** 20 Minutes
* **Arena Dimensions:** Outdoor terrain course covering approximately **53 m × 34 m**
* **Base Station (Home):** Marked **2 m × 2 m** zone
* **GNSS Standard:** WGS 84 datum (Coordinates given in Latitude / Longitude format)
* **Max Speed Limit:** 1.5 m/s on course
* **Autonomy Level:** 100% Fully Autonomous (Zero teleoperation or remote wireless computation allowed after start signal)
* **Safety:** Hardware kill-switch must always be accessible and functional.

---

## 🎯 2. Mission Phases Breakdown

```
[Base Station (2m×2m)] ──(Depart on Start)──> [Obstacle Field]
                                                      │
                                           (Avoid obstacles without contact)
                                                      │
                                                      ▼
                                           [GPS Point 1 (±1.5m)]
                                                      │
                                           (Tree Vitality Detection: Alive/Dead)
                                                      │
                                                      ▼
                                           [GPS Point 2 (±1.5m)]
                                                      │
                                           (360° Panoramic Image Capture)
                                                      │
                                                      ▼
                                      [Return to Base Station (2m×2m)]
                                                      │
                                           (Stop inside zone for Time Bonus)
```

---

### 📍 Phase 1: Departure, Obstacle Avoidance & Tree Vitality Detection
1. **Departure:** Rover autonomously departs the 2 m × 2 m Base Station upon receiving a single start signal.
2. **Obstacle Avoidance:**
   * Detect and navigate around all terrain obstacles (rocks, ditches, posts) between Base Station and GPS Point 1.
   * **Zero physical contact** with any obstacle.
3. **Arrival at GPS Point 1:** Navigate to within **1.5 m** tolerance of GPS Point 1.
4. **Tree Vitality Detection:**
   * At GPS Point 1, autonomously inspect the designated tree.
   * Determine whether the tree is **ALIVE** or **DEAD** using onboard visual/thermal/multispectral sensing.
   * Log the verdict with timestamp and confidence score.

---

### 📷 Phase 2: Navigation to GPS Point 2 & Panoramic Imaging
1. **Waypoint Traversal:** Autonomously navigate from GPS Point 1 to GPS Point 2.
2. **Arrival at GPS Point 2:** Stop within **1.5 m** tolerance of GPS Point 2.
3. **360° Panoramic Imaging:**
   * Capture a full 360° panoramic image of the surrounding terrain.
   * **Minimum Resolution:** 1920 × 1080 px.
   * **Horizon Level:** Level within ±5°.
   * **Image Quality:** Clean stitching, no severe motion blur.

---

### 🏁 Phase 3: Return to Base Station & Time Bonus
1. **Return Navigation:** Autonomously compute path back to the Base Station.
2. **Precision Stop:** Come to a complete stop inside the **2 m × 2 m** Base Station zone.
3. **Time Bonus Tiers:**
   * 🥇 **Tier 1:** Complete within **< 15 minutes** (Highest Bonus)
   * 🥈 **Tier 2:** Complete within **< 18 minutes**
   * 🥉 **Tier 3:** Complete within **< 20 minutes**

---

## 📁 3. Mandatory Post-Mission Deliverables (Within 10 Minutes)
Within 10 minutes of mission completion, the team must submit 3 files generated onboard:
1. `tree_verdict.log` — Tree Vitality Detection Verdict (Alive/Dead, timestamps, sensor data summary).
2. `panorama_360.jpg` — Full 360° Panoramic Image (min 1080p, horizon ±5°).
3. `onboard_run.log` — Full execution trace:
   * GPS path followed (lat/lon / timestamps)
   * Obstacle detection events & avoidance maneuvers
   * State transitions (`IDLE` → `NAV_POINT_1` → `TREE_ANALYSIS` → `NAV_POINT_2` → `PANORAMA` → `RETURNING` → `DONE`)

---

## ⚙️ 4. Software Architecture Mapping (autonomous_meer)

| Mission Requirement | Package Responsible | Node / Module |
|---|---|---|
| **GPS to Local Coordinates** | `meer_navigation` | `nav2_gps_waypoint_follower` / `robot_localization` NavSat |
| **Obstacle Avoidance** | `meer_navigation` / `meer_perception` | LiDAR Costmap 2D (`local_costmap`) + `obstacle_detector` |
| **Tree Vitality Classifier** | `meer_perception` | `tree_vitality_classifier` (Vision / HSV / NDVI / AI model) |
| **360° Panoramic Imaging** | `meer_perception` | `panorama_generator` (Rotational scan or wide-angle stitcher) |
| **Mission State Machine & Logging** | `meer_mission_control` | `mission_control_node` (FSM + Run logger) |
| **Drive & Low-level Actuation** | `meer_bringup` (Sim) / `meer_hardware` (Real) | Gazebo DiffDrive / Cytron MDDS30 via micro-ROS |

---

## 🔒 5. Hard Operational Constraints
* **No Remote Teleoperation:** Base station visibility of the arena will be completely blocked.
* **100% Onboard Compute:** No WiFi/radio offloading of vision or path planning algorithms.
* **Pre-loading Allowed:** GPS coordinates for Point 1 and Point 2 may be loaded into YAML before run start.
* **Speed Cap:** Max velocity capped at **1.5 m/s** (`linear.x.max_velocity: 1.5`).
* **Kill Switch:** Hardware E-Stop must instantly halt power to the motors.
