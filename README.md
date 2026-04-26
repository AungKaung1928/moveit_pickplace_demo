# MoveIt2 Smart Pick-and-Place Demo

Vision-guided pick-and-place on the Franka Panda arm — combining OpenCV perception, PyTorch object classification, C++ workspace validation, and MoveIt2 motion planning in a full autonomous loop.

No real hardware required. Everything runs in simulation with `ros2_control` mock hardware.

---

## Demo

The robot continuously scans a synthetic workspace, identifies coloured objects, validates reachability, and sorts them into colour-coded bins — fully autonomously.

```
SCANNING → PRE_GRASPING → GRASPING → LIFTING → PLACING → HOMING → (repeat)
```

---

## Architecture

```
camera_simulator ──► vision_detector ──► workspace_validator (C++) ──► smart_pick_place
  (synthetic          (HSV + ArUco         (cylindrical reachability      (7-state FSM,
   640×480 cam)        + YOLOv8n overlay)   filter → /validated_targets)   colour sorting)
       │                                                                         │
       └──► depth_estimator                                               MoveIt2 /move_action
              (YOLOv8n → bbox centre                                             │
               → pinhole projection                                  ros2_control mock hardware
               → /object_positions)                                    (panda_arm_controller)
```

YOLOv8n (COCO weights) runs on every frame in both `vision_detector` and `depth_estimator`. 0 COCO detections on synthetic colour shapes is expected — see [Domain Gap Analysis](#domain-gap-analysis). The actual pick-and-place loop is driven by HSV + ArUco detections, not YOLO.

### Packages

| Package | Language | Role |
|---------|----------|------|
| `simple_moveit_demo` | Python | Vision pipeline, FSM controller, grasp planner |
| `moveit_grasp_utils` | C++ | Geometric workspace reachability validator |

### Key Files

| File | What it does |
|------|-------------|
| `camera_simulator.py` | Publishes synthetic 640×480 top-down camera with coloured objects + ArUco markers |
| `vision_detector.py` | Runs YOLOv8n on each frame (overlay) + HSV colour segmentation + ArUco pose estimation → `/detected_objects` |
| `shape_classifier.py` | `YOLODetector` wrapper (YOLOv8n via ultralytics) + `geometric_classify` heuristic fallback |
| `depth_estimator.py` | YOLO bbox centre → 3D via pinhole model → `geometry_msgs/PointStamped` on `/object_positions` |
| `export_model.py` | Exports YOLOv8n to ONNX (640×480 fixed input), verifies inference |
| `benchmark.py` | Runs ONNX model 100 iterations, reports mean/std latency and FPS (simulates Jetson TensorRT profiling) |
| `grasp_planner.py` | Computes pre-grasp → grasp → lift → bin pose sequence from object position |
| `smart_pick_place.py` | 7-state FSM — sends `MoveGroup` action goals, handles recovery, colour-sorts to bins |
| `workspace_validator_node.cpp` | Validates detected positions against Panda cylindrical reachability bounds, publishes to RViz |

---

## Prerequisites

### System dependencies
```bash
sudo apt install ros-humble-moveit ros-humble-ros2-control \
    ros-humble-ros2-controllers ros-humble-moveit-ros-move-group \
    ros-humble-moveit-planners-ompl ros-humble-franka-description \
    python3-cv-bridge
```

### Python dependencies

> **Important:** `cv_bridge` is compiled against NumPy 1.x. NumPy 2.x causes a fatal `_ARRAY_API not found` error at startup. Downgrade first.

```bash
pip install "numpy<2"
pip install ultralytics onnxruntime
```

Verify:
```bash
python3 -c "import numpy; print(numpy.__version__)"   # must be 1.x
```

---

## Build

```bash
mkdir -p ~/moveit_project_ws/src
cd ~/moveit_project_ws/src
git clone https://github.com/AungKaung1928/moveit_pickplace_demo.git simple_moveit_demo

cd ~/moveit_project_ws
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

---

## Running — Three Terminals

Open three separate terminal tabs. Source the workspace in each one before running any command:

```bash
source ~/moveit_project_ws/install/setup.bash
```

---

### Terminal 1 — Full Stack Launch

This single command starts everything: MoveIt2, ros2_control mock hardware, the complete vision pipeline (camera → detector → C++ validator), the FSM controller, and RViz.

```bash
cd ~/moveit_project_ws
source install/setup.bash
ros2 launch simple_moveit_demo smart_pick_place.launch.py
```

**What starts:**

| Node | What it does |
|------|-------------|
| `move_group` | MoveIt2 motion planning server (OMPL RRTConnect) |
| `robot_state_publisher` | Publishes Panda URDF transforms |
| `ros2_control_node` | Mock hardware controller |
| `panda_arm_controller` | JointTrajectory controller (mock) |
| `joint_state_broadcaster` | Publishes `/joint_states` |
| `camera_simulator` | Synthetic 640×480 top-down camera |
| `vision_detector` | HSV + ArUco + YOLOv8n detection |
| `workspace_validator` | C++ reachability filter |
| `smart_pick_place` | 7-state FSM controller |
| `rviz2` | Visualisation |

**Startup sequence in T1 logs:**

```
[camera_simulator]   Camera simulator started — 640×480 @ 10 Hz
[vision_detector]    Vision detector initialised — waiting for camera frames...
[smart_pick_place]   Still waiting for MoveGroup action server...
                     ...  (move_group finishes loading ~5–10 s)
[smart_pick_place]   MoveGroup connected — entering SCANNING state.
[vision_detector]    Detected 8 objects — best: red_cube @ (0.418, 0.119, 0.000)
[workspace_validator] Validated 8/8 objects as reachable
[smart_pick_place]   [STATE] → PRE_GRASPING
[move_group]         sending trajectory to panda_arm_controller
```

The arm begins moving in RViz after the ~6-second warm-up. Each pick-and-place cycle takes 30–60 seconds depending on planning time.

**Expected normal log lines:**
- `YOLO: 2 object(s) detected` — ArUco markers on synthetic objects are misclassified as "stop sign" by COCO weights. This is expected (domain gap).
- `Validated 8/8 objects as reachable` — all 4 HSV detections + 4 ArUco poses pass the reachability filter.
- `[STATE] → GRASPING / LIFTING / PLACING / HOMING` — FSM cycling normally.

**Shutdown notes:**
- `rviz2 exit code -11` on Ctrl+C — known MoveIt2 Humble segfault in destructor, harmless.
- `move_group SIGKILL after 10 s` — also a known Humble shutdown issue, harmless.

---

### Terminal 2 — Depth Estimator (Optional Standalone Node)

The depth estimator runs YOLO independently and projects detected bounding box centres to 3D world coordinates using the camera's known geometry. It is not required for the pick-and-place FSM (which uses `/validated_targets` from the C++ validator instead), but it demonstrates the pinhole projection pipeline.

```bash
cd ~/moveit_project_ws
source install/setup.bash
ros2 run simple_moveit_demo depth_estimator
```

**What you see:**

```
[depth_estimator] Depth estimator ready — YOLOv8n bbox → 3D position on /object_positions
[depth_estimator] YOLO [stop sign 0.42] → 3D (0.479, 0.197, 0.000) on /object_positions
[depth_estimator] YOLO [stop sign 0.35] → 3D (0.505, -0.097, 0.000) on /object_positions
```

- 2 detections per frame are normal — the two circle-shaped objects (green + yellow) trigger "stop sign" matches in COCO.
- Positions reported match `SCENE_OBJECTS` world coordinates in `camera_simulator.py`.
- If you see `No YOLO detections` for the rectangle objects — that is the documented domain gap.

**Requires T1 to be running** (subscribes to `/camera/image_raw` and `/camera/camera_info`).

---

### Terminal 3 — Monitoring

Use these commands in T3 to inspect the pipeline while T1 is running. Run each in a separate tab or kill one before running another.

**FSM state (most useful — shows what the arm is doing):**
```bash
ros2 topic echo /pick_place_status
```
Expected output cycling:
```
data: SCANNING
data: PRE_GRASPING
data: GRASPING
data: LIFTING
data: PLACING
data: HOMING
data: SCANNING
```

**MoveGroup result codes:**
```bash
ros2 topic echo /pick_place_debug
```
`error_code=1` = success. Any other value = planning failed (robot attempts recovery automatically).

**Raw vision detections:**
```bash
ros2 topic echo /detected_objects
```
Shows all detected markers (colour + shape label, 3D position, confidence).

**Workspace-validated targets (what the FSM actually picks from):**
```bash
ros2 topic echo /validated_targets
```

**YOLO 3D positions (from depth_estimator, only if T2 is running):**
```bash
ros2 topic echo /object_positions
```

**Topic list — see everything publishing:**
```bash
ros2 topic list
ros2 topic hz /camera/image_raw      # should be ~10 Hz
ros2 topic hz /detected_objects      # should be ~10 Hz
```

---

## What to See in RViz

| Element | What it means |
|---------|--------------|
| Coloured spheres on the table | Detected objects — red / green / blue / yellow |
| Orange/green cylinders | Panda reachable workspace bounds from C++ validator |
| Arm moving autonomously | FSM executing PRE_GRASP → GRASP → LIFT → PLACE cycle |
| Arm returns to centre | HOMING state between cycles |

If the arm is not moving: check T1 logs for `[STATE] → SCANNING` — if it never leaves SCANNING, no `/validated_targets` are being published. This usually means the vision pipeline crashed (check for `_ARRAY_API` or `Dictionary_get` errors near the top of T1 output).

---

## Technical Stack

| Component | Technology |
|-----------|-----------|
| Framework | ROS2 Humble |
| Motion planning | MoveIt2 + OMPL (RRTConnect) |
| Robot simulation | `ros2_control` with `mock_components` hardware |
| Language | Python 3.10 + C++17 |
| Vision | OpenCV 4.8 — HSV segmentation, ArUco detection, contour analysis |
| Detection model | YOLOv8n (ultralytics) — COCO pretrained, ONNX export + CPU benchmark |
| 3D estimation | Pinhole projection: YOLO bbox centre → robot frame via known camera geometry |
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
| Acceleration scaling | 0.3 | 30% of max acceleration |
| Position tolerance | 0.04–0.06 m | Relaxed for simulation |

---

## Workspace and Bin Layout

```
Panda reachable zone: 0.18 m – 0.82 m radius, up to 0.98 m height

Scene objects (camera_simulator.py):
  red    rect   @ (0.42,  0.12)
  green  circle @ (0.50, -0.10)
  blue   rect   @ (0.33, -0.18)
  yellow circle @ (0.48,  0.20)

Bin positions (panda_link0 frame):
  red    → ( 0.55, -0.28, 0.30)   right-far
  green  → ( 0.55,  0.28, 0.30)   left-far
  blue   → ( 0.30, -0.35, 0.30)   right-near
  yellow → ( 0.30,  0.35, 0.30)   left-near
```

---

## Domain Gap Analysis

YOLOv8n is pretrained on **COCO** (118k real-world images). The inference target here is a **synthetic top-down camera** rendering flat coloured shapes on a dark table. The model returns 0 detections on rectangle objects and misclassifies circle objects as "stop signs" — this is expected and documented.

### Three gap factors

| Factor | COCO training distribution | Synthetic camera |
|--------|---------------------------|-----------------|
| **Lighting** | Natural and indoor lighting with shadows, highlights, gradients | Uniform flat illumination, no shadows, no specular reflections |
| **Viewpoint** | Objects at human eye-level or slight top-down angle | Strict orthographic top-down view — objects appear as 2D projections with no depth cues |
| **Texture** | Surface texture, logos, wear patterns — rich high-frequency detail | Solid-colour flat fills, no surface texture |

### Why "stop sign" detections appear

The ArUco markers (black/white squares) overlaid on circle objects happen to match the COCO "stop sign" class at low confidence (~0.25–0.45). This is an artefact of the octagonal black/white pattern similarity. It does not affect the pick-and-place FSM, which uses HSV + ArUco detections via `/validated_targets`.

### Closing the gap for real hardware

1. Fine-tune YOLOv8n on a labelled dataset of real top-down images from the deployment camera (50–200 images is usually sufficient)
2. Use domain randomisation during synthetic data generation (vary lighting, add noise, randomise backgrounds)
3. Measure mAP on a held-out real-image validation set before deploying

The `export_model.py` + `benchmark.py` tools establish the latency baseline. On a Jetson Orin with TensorRT FP16 the same model runs at ~5–10× lower latency than the CPU numbers `benchmark.py` reports.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `AttributeError: _ARRAY_API not found` | NumPy 2.x installed; cv_bridge compiled against 1.x | `pip install "numpy<2"` then rebuild |
| `AttributeError: module 'cv2.aruco' has no attribute 'Dictionary_get'` | OpenCV 4.8+ removed old ArUco API | Already fixed in this repo; update if you see an old clone |
| Arm stuck in SCANNING state | Vision pipeline not publishing `/detected_objects` | Check T1 for startup errors; verify camera_simulator and vision_detector started cleanly |
| `rviz2 exit code -11` on shutdown | Known MoveIt2 Humble destructor bug | Harmless — only affects shutdown |
| `move_group SIGKILL` on shutdown | Humble shutdown race condition | Harmless — only affects shutdown |
| YOLO returns 0 detections | Domain gap: COCO weights vs synthetic top-down shapes | Expected — HSV segmentation drives the FSM, not YOLO |

---

## Project Purpose

Portfolio project demonstrating physical AI skills:
- **Perception pipeline**: synthetic camera → HSV + ArUco detection → PyTorch shape classification
- **Spatial reasoning**: C++ node validates detections against robot kinematic bounds before committing to a grasp
- **Motion planning**: MoveGroup action client with OMPL, position-constrained Cartesian goals
- **System integration**: multi-language ROS2 (Python + C++), multi-node FSM, `ros2_control` hardware abstraction

Next step: swap `mock_components` for a Gazebo or real Panda interface — the perception and planning stack requires no changes.
