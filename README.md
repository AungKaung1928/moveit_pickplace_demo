# MoveIt2 Pick-and-Place Demo

Franka Panda arm picks 7 red balls from a table and places them into a box — fully autonomous, simulation-only, no real hardware required.

## Architecture

```
camera_simulator ──► vision_detector ──► workspace_validator (C++) ──► smart_pick_place (FSM)
  (synthetic              (HSV + ArUco          (reachability filter)       GRASPING → PLACING
   top-down cam)           detection)                                        → repeat → DONE
                                                                                  │
                                                                           MoveIt2 /move_action
                                                                           (OMPL RRTConnect)
                                                                                  │
                                                                        ros2_control mock hardware
```

## Key Technical Decisions

| Decision | Why |
|---|---|
| C++ workspace validator | Real-time reachability filtering — must not block the Python FSM |
| Python FSM + MoveGroup action | High-level orchestration — async goal sending with `threading.Event` |
| Ghost ball marker pattern | Ball stays visible during approach (RViz `Marker`), collision object removed so MoveIt can plan to grasp pose, reappears in box slot after placement |
| Fixed yaw=0° for all grasps | Arm maintains same wrist orientation grasp→place→grasp — OMPL finds simpler joint paths |
| No `AttachedCollisionObject` | Eliminated: caused green sphere artifact on gripper + panda_link7 self-collision failures |

## FSM States

```
IDLE → GRASPING → PLACING → (next ball) → ... → DONE
                       └── HOMING (recovery only, on planning failure)
```

## Dependencies

**ROS2 Humble + MoveIt2**
```bash
sudo apt install ros-humble-moveit ros-humble-ros2-control \
    ros-humble-ros2-controllers ros-humble-moveit-ros-move-group \
    ros-humble-moveit-planners-ompl python3-cv-bridge
```

**Python**
```bash
pip install "numpy<2"   # cv_bridge requires NumPy 1.x
pip install ultralytics onnxruntime
```

## Build and Launch

```bash
mkdir -p ~/moveit_project_ws/src
cd ~/moveit_project_ws/src
git clone https://github.com/AungKaung1928/moveit_pickplace_demo.git .

cd ~/moveit_project_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

ros2 launch simple_moveit_demo smart_pick_place.launch.py
```

RViz opens automatically. The arm starts moving after ~5–10 s MoveIt2 warm-up.

## What to See in RViz

- 7 red balls on the table, 1 orange box
- Ball stays visible (red sphere marker) while arm approaches
- Ball disappears the moment arm reaches grasp position
- Arm moves directly to box — no intermediate home return
- Ball reappears inside the box, stacked in a 2×2 grid (layer 2 for balls 5–7)
- Repeats until all 7 balls are placed

## Packages

| Package | Language | Role |
|---|---|---|
| `simple_moveit_demo` | Python | Vision pipeline, FSM controller, scene manager, grasp planner |
| `moveit_grasp_utils` | C++ | Geometric workspace reachability validator |

## Planning Parameters

| Parameter | Value |
|---|---|
| Planner | RRTConnect (OMPL) |
| Planning attempts | 20 |
| Planning time | 15 s |
| Velocity scaling | 0.3 |
| Position tolerance (grasp) | 0.04 m |
| Position tolerance (place) | 0.06 m |

## Planned Improvements

- Swap `mock_components` for Gazebo Ignition with Panda plugin — perception and planning stack unchanged
- Train YOLOv8n on real top-down images to close domain gap (currently COCO weights on synthetic shapes)
- Export grasp policy to ONNX + TensorRT for Jetson edge deployment
- Add camera–LiDAR fusion for 3D object detection
