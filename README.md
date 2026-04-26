# MoveIt2 Smart Pick-and-Place Demo

Vision-guided pick-and-place on the Franka Panda arm — combining OpenCV perception, PyTorch object classification, C++ workspace validation, and MoveIt2 motion planning in a full autonomous loop.

No real hardware required. Everything runs in simulation with `ros2_control` mock hardware.

---

## Demo

The robot continuously scans a synthetic workspace, identifies coloured objects, validates reachability, and sorts them into colour-coded bins — fully autonomously.

```
SCANNING → PRE_GRASPING → GRASPING → LIFTING → PLACING → HOMING → (repeat)
```

**Monitor the FSM live:**
```bash
ros2 topic echo /pick_place_status
```

---

## Architecture

```
camera_simulator ──► vision_detector ──► workspace_validator (C++) ──► smart_pick_place
  (synthetic          (OpenCV + ArUco      (cylindrical reachability      (7-state FSM,
   640×480 cam)        + PyTorch MLP)       filter → /validated_targets)   colour sorting)
                                                                                │
                                                                         MoveIt2 /move_action
                                                                                │
                                                                    ros2_control mock hardware
                                                                      (panda_arm_controller)
```

### Packages

| Package | Language | Role |
|---------|----------|------|
| `simple_moveit_demo` | Python | Vision pipeline, FSM controller, grasp planner |
| `moveit_grasp_utils` | C++ | Geometric workspace reachability validator |

### Key Files

| File | What it does |
|------|-------------|
| `camera_simulator.py` | Publishes synthetic 640×480 top-down camera with coloured objects + ArUco markers |
| `vision_detector.py` | HSV multi-colour detection + ArUco pose estimation + contour shape features |
| `shape_classifier.py` | PyTorch 2-layer MLP — trains at startup on synthetic data, classifies `cube / cylinder / irregular` |
| `grasp_planner.py` | Computes pre-grasp → grasp → lift → bin pose sequence from object position |
| `smart_pick_place.py` | 7-state FSM — sends `MoveGroup` action goals, handles recovery, colour-sorts to bins |
| `workspace_validator_node.cpp` | Validates detected positions against Panda cylindrical reachability bounds, publishes to RViz |

---

## Quick Start

### 1. Clone and build

```bash
mkdir -p ~/moveit_project_ws/src
cd ~/moveit_project_ws/src
git clone https://github.com/AungKaung1928/moveit_pickplace_demo.git simple_moveit_demo

cd ~/moveit_project_ws
rosdep install --from-paths src --ignore-src -r -y
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu

colcon build --symlink-install
source install/setup.bash
```

### 2. Launch (single command)

```bash
ros2 launch simple_moveit_demo smart_pick_place.launch.py
```

This starts:
- MoveIt2 `move_group` + `robot_state_publisher`
- `ros2_control` mock hardware + `panda_arm_controller` + `joint_state_broadcaster`
- RViz2 with custom config
- Full vision pipeline (camera → detector → C++ validator)
- Smart pick-place FSM (auto-starts after 6 s warm-up)

### 3. Monitor

```bash
# FSM state
ros2 topic echo /pick_place_status

# MoveGroup result codes (1 = success)
ros2 topic echo /pick_place_debug

# Vision detections
ros2 topic echo /detected_objects

# Workspace-validated target poses
ros2 topic echo /validated_targets
```

---

## What to see in RViz

| Element | What it means |
|---------|--------------|
| Coloured spheres on table | Detected objects (red / green / blue / yellow) |
| Orange/green cylinders | Panda reachable workspace bounds from C++ validator |
| Arm moving autonomously | FSM executing PRE_GRASP → GRASP → LIFT → PLACE cycle |

---

## Technical Stack

| Component | Technology |
|-----------|-----------|
| Framework | ROS2 Humble |
| Motion planning | MoveIt2 + OMPL (RRTConnect) |
| Robot simulation | `ros2_control` with `mock_components` hardware |
| Language | Python 3.10 + C++17 |
| Vision | OpenCV 4.5 — HSV segmentation, ArUco detection, contour analysis |
| ML | PyTorch 2.x — 2-layer MLP shape classifier (trains at startup) |
| Robot model | Franka Emika Panda (7-DOF) |
| Visualisation | RViz2 + MarkerArray |

---

## Planning Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Planner | RRTConnect (OMPL) | Default MoveIt2 pipeline |
| Planning attempts | 20 | Per motion segment |
| Planning time | 15 s | Per motion segment |
| Velocity scaling | 0.3 | 30% of max speed |
| Position tolerance | 0.04–0.06 m | Relaxed for simulation |

## Workspace & Bin Layout

```
Panda reachable zone: 0.18 m – 0.82 m radius, up to 0.98 m height

Bin positions (panda_link0 frame):
  red    → ( 0.55, -0.28, 0.30)   right-far
  green  → ( 0.55,  0.28, 0.30)   left-far
  blue   → ( 0.30, -0.35, 0.30)   right-near
  yellow → ( 0.30,  0.35, 0.30)   left-near
```

---

## Project Purpose

Portfolio project demonstrating physical AI skills:
- **Perception pipeline**: synthetic camera → HSV + ArUco detection → PyTorch shape classification
- **Spatial reasoning**: C++ node validates detections against robot kinematic bounds before committing to a grasp
- **Motion planning**: MoveGroup action client with OMPL, position-constrained Cartesian goals
- **System integration**: multi-language ROS2 (Python + C++), multi-node FSM, `ros2_control` hardware abstraction

Next step: swap `mock_components` for a Gazebo or real Panda interface — the perception and planning stack requires no changes.
