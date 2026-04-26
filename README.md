# MoveIt2 Smart Pick-and-Place Demo

Vision-guided pick-and-place using MoveIt2, OpenCV, ArUco markers, PyTorch object classification, and a C++ workspace validator — on the Franka Panda arm.

---

## Architecture

```
camera_simulator ──► vision_detector ──► workspace_validator (C++) ──► smart_pick_place
  (synthetic          (OpenCV + ArUco      (reachability check,          (FSM: 7 states,
   top-down cam)       + PyTorch shape      publishes valid targets)      colour sorting)
                       classifier)
                            │
                            ▼
                       /detected_objects (RViz markers)
```

### Packages
| Package | Language | Role |
|---------|----------|------|
| `simple_moveit_demo` | Python | Vision pipeline, FSM controller, grasp planner |
| `moveit_grasp_utils` | C++ | Geometric workspace reachability validator |

### Key Files
| File | What it does |
|------|-------------|
| `camera_simulator.py` | Publishes synthetic 640×480 top-down camera with 4 coloured objects + ArUco markers |
| `vision_detector.py` | HSV multi-colour detection + ArUco pose estimation + contour shape features |
| `shape_classifier.py` | PyTorch MLP — trains at startup, classifies `cube / cylinder / irregular` |
| `grasp_planner.py` | Computes pre-grasp → grasp → lift → bin pose sequence from detection |
| `smart_pick_place.py` | 7-state FSM with colour-based bin sorting (red/green/blue/yellow → separate bins) |
| `workspace_validator_node.cpp` | Validates detected positions against Panda cylindrical workspace bounds |

---

## Running the Demo

### New smart demo — single command

```bash
cd ~/moveit_project_ws
source install/setup.bash
ros2 launch simple_moveit_demo smart_pick_place.launch.py
```

This starts everything at once:
- MoveIt2 move_group + RViz
- Synthetic camera (no hardware needed)
- Vision detector + C++ workspace validator
- Smart pick-place FSM controller

### Original simple demo — two terminals (still works)

**Terminal 1:**
```bash
cd ~/moveit_project_ws && source install/setup.bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

**Terminal 2:**
```bash
cd ~/moveit_project_ws && source install/setup.bash
ros2 run simple_moveit_demo pick_place_demo
```

---

## What to observe

**In RViz:**
- Green/orange cylinders = Panda reachable workspace bounds (from C++ validator)
- Coloured spheres = detected objects (red/green/blue/yellow)
- Motion Planning panel = robot arm trajectory preview

**In terminal:**
```
[vision_detector] Detected 4 objects — best: red_cube @ (0.420, 0.120, 0.000)
[workspace_validator] Validated 3/4 objects as reachable
[smart_pick_place] [STATE] → PRE_GRASPING
[smart_pick_place] [STATE] → GRASPING
[smart_pick_place] [GRIPPER] CLOSE
[smart_pick_place] [STATE] → LIFTING
[smart_pick_place] [STATE] → PLACING
[smart_pick_place] Cycle complete — placed [red_cube] in [red] bin.
[smart_pick_place] [STATE] → SCANNING
```

**Topics to monitor:**
```bash
ros2 topic echo /pick_place_status      # current FSM state
ros2 topic echo /detected_objects       # vision detections
ros2 topic echo /validated_targets      # workspace-validated poses
ros2 topic echo /detection_image        # annotated camera view
```

---

## Build

```bash
mkdir -p ~/moveit_project_ws/src
cd ~/moveit_project_ws/src
git clone git@github.com:AungKaung1928/moveit_pickplace_demo.git simple_moveit_demo

cd ~/moveit_project_ws
rosdep install --from-paths src --ignore-src -r -y
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu

colcon build --packages-select moveit_grasp_utils simple_moveit_demo
source install/setup.bash
```

---

## Technical Stack

| Component | Technology |
|-----------|-----------|
| Framework | ROS2 Humble |
| Motion planning | MoveIt2 + OMPL (RRTConnect) |
| Language | Python 3.10 + C++17 |
| Vision | OpenCV 4.5 (HSV, ArUco, contour analysis) |
| ML | PyTorch 2.x — MLP shape classifier |
| Robot | Franka Emika Panda (7-DOF) |
| Visualisation | RViz2 + MarkerArray |

## Parameters (config/demo_params.yaml)

```yaml
smart_pick_place:
  velocity_scaling: 0.15      # 15% speed (safe for demo)
  planning_attempts: 12

workspace_validator:
  max_reach:  0.82             # metres from base
  min_reach:  0.18
  max_height: 0.98
```

## Bin Layout (config/demo_params.yaml)

```
red   → (0.55, -0.28, 0.30)  right side
green → (0.55,  0.28, 0.30)  left side
blue  → (0.30, -0.35, 0.30)  centre-right
yellow→ (0.30,  0.35, 0.30)  centre-left
```
