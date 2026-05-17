# CLAUDE.md — Physical AI / Embodied AI Portfolio Projects

## Who I Am

Aung Kaung Myat — building a physical AI / embodied AI career.
2-year target: land a role at a physical AI company (manipulation, mobile robotics, robot learning).
All projects are simulation-only (Gazebo / RViz) unless hardware is explicitly mentioned.
Portfolio lives at GitHub: AungKaung1928.

---

## Language Roles — Hard Rules

### C++ (primary for robotics logic)
Use C++ for:
- All ROS2 nodes that run in real-time or near-real-time loops
- State machines and behavior controllers
- Geometry validators, workspace checkers, collision filters
- Custom controllers and trajectory processors
- Any node that subscribes to sensor data at high frequency (>10 Hz)
- MoveIt2 MoveGroupInterface calls
- Point cloud processing (PCL)

### Python (perception, ML, orchestration)
Use Python for:
- Perception pipelines (OpenCV, cv_bridge, ArUco, HSV)
- PyTorch model definition, training, and inference
- Synthetic data / scene generators
- High-level task planners and mission managers
- ROS2 launch files (always Python launch files, never XML)
- Test scripts and data logging utilities

### Never mix roles
Do not write performance-critical sensor loops in Python.
Do not write PyTorch model training in C++.

---

## ROS2 Package Structure

Every project has at least two packages in one git repo:

```
<project_ws>/
  src/
    <project_name>/          # ament_python — perception + ML + orchestration
      <project_name>/
        __init__.py
        <node>.py
      resource/
      package.xml
      setup.py
      setup.cfg
    <project_name>_utils/    # ament_cmake — C++ real-time nodes
      include/<project_name>_utils/
        <header>.hpp
      src/
        <node>.cpp
      CMakeLists.txt
      package.xml
    <project_name>_msgs/     # ament_cmake — custom msgs/srvs if needed
      msg/
      srv/
      CMakeLists.txt
      package.xml
  launch/                    # shared launch files (put in Python package)
  config/                    # YAML param files
  README.md
```

---

## C++ Coding Standards

### Node structure
- Inherit from `rclcpp::Node` directly for simple nodes
- Use `rclcpp_lifecycle::LifecycleNode` for nodes that manage hardware or heavy resources
- Constructor takes `const rclcpp::NodeOptions&` for composability
- Declare all parameters in constructor with `declare_parameter<T>(name, default)`

### Memory
- No raw `new` / `delete` — use `std::make_shared`, `std::make_unique`
- ROS2 handles: `rclcpp::Publisher<T>::SharedPtr`, `rclcpp::Subscription<T>::SharedPtr`
- Callbacks: prefer lambdas with explicit captures over `std::bind`

### Threading
- Use `rclcpp::spin` for single-threaded nodes
- Use `rclcpp::executors::MultiThreadedExecutor` + `rclcpp::CallbackGroup` when a node has multiple independent callback chains
- Protect shared state with `std::mutex` + `std::lock_guard`

### Geometry and math
- Use `Eigen3` for all matrix / vector math — never roll your own
- Use `tf2_eigen` for converting between ROS messages and Eigen types
- Use `geometry_msgs::msg::PoseStamped` with proper frame_id — never assume a frame

### Style
- `snake_case` for variables, functions, file names
- `PascalCase` for class names
- `SCREAMING_SNAKE_CASE` for constants
- Headers: `#pragma once` (not include guards)
- Include order: C++ std → third-party → ROS2 → project-local
- No `using namespace std` or `using namespace rclcpp` in headers

### CMakeLists.txt
- `ament_cmake` as build type
- Export dependencies with `ament_target_dependencies`
- Install targets: nodes → `lib/${PROJECT_NAME}`, headers → `include`
- Always call `ament_package()` last

---

## Python Coding Standards

### General
- Type hints on all function signatures
- `rclpy.node.Node` subclass per node file
- `main()` entry point guarded by `if __name__ == '__main__'`
- `setup.py` entry_points console_scripts for every node

### ROS2 patterns
- Declare parameters with `self.declare_parameter(name, default)` in `__init__`
- Read params with `self.get_parameter(name).value`
- Use `self.get_logger().info/warn/error` — never `print()`
- Use `self.create_timer` for periodic callbacks, not `while True` loops

### PyTorch
- Models are small (2–4 layers) — target CPU inference, no GPU requirement
- Training data is synthetic and generated at startup if no checkpoint exists
- Save / load checkpoints with `torch.save` / `torch.load`
- Wrap inference in `torch.no_grad()` always
- Normalize inputs — document the normalization scheme in the model class

### OpenCV / Perception
- Always check image dimensions before processing
- Convert ROS image messages with `cv_bridge.CvBridge`
- HSV thresholds go in YAML config, not hardcoded
- ArUco: use `cv2.aruco.DetectorParameters` (OpenCV 4.7+ API)

---

## Architecture Patterns

### Finite State Machine (FSM)
Use an explicit FSM for any behavior that has more than 3 stages.
- Define states as a Python `enum.Enum` or C++ `enum class`
- One `execute_<state>()` method per state
- Log every state transition at INFO level
- FSM tick runs in a ROS2 timer callback, not a blocking loop

### Perception → Planning → Control pipeline
Standard node graph for manipulation projects:
```
[Camera/Sensor Node] → /raw_detections
      ↓
[Perception Node (Python)] → /detected_objects (MarkerArray or custom msg)
      ↓
[Validator Node (C++)] → /validated_targets (PoseArray)
      ↓
[Planner / FSM Node (Python or C++)] → /trajectory or MoveIt2 API
      ↓
[Controller / Executor Node (C++)]
```

### Parameters over constants
Nothing scene-specific is hardcoded. Bin positions, speed limits, thresholds, frame names — all go in `config/<node_name>.yaml` and loaded via `declare_parameter`.

---

## Simulation Setup

- Gazebo Harmonic (Ignition) preferred; Gazebo Classic acceptable for older projects
- Always provide a `.world` or SDF file in `worlds/`
- RViz2 config saved as `rviz/<project_name>.rviz` and loaded from launch file
- Simulated sensors: use Gazebo plugins (camera, lidar, IMU) — no fake publishers unless prototyping
- Robot models: use `xacro` + `urdf` in a `description/` directory

---

## Key Libraries and Frameworks

| Library | Role | Language |
|---|---|---|
| ROS2 Humble | middleware | C++ / Python |
| MoveIt2 | manipulation planning | C++ (MoveGroupInterface) |
| Nav2 | autonomous navigation | C++ / Python |
| Gazebo Harmonic | simulation | — |
| OpenCV 4.x | image processing | Python |
| PyTorch (CPU) | ML inference + training | Python |
| Eigen3 | linear algebra | C++ |
| PCL | point cloud processing | C++ |
| tf2 / tf2_eigen | coordinate transforms | C++ / Python |
| cv_bridge | ROS ↔ OpenCV bridge | Python |
| visualization_msgs | RViz markers | C++ / Python |
| rclcpp_action | action server/client | C++ |
| rclpy_action | action server/client | Python |

---

## What NOT to Do

- No `rospy` — this is ROS2 only
- No XML launch files — Python launch files only
- No hardcoded topic names, frame names, or thresholds in source code
- No `print()` in ROS2 nodes — use `get_logger()`
- No blocking `while rclpy.ok()` loops inside node methods — use timers
- No fake hardware publishers as a permanent solution — use Gazebo plugins
- No `requirements.txt` without a matching `package.xml` `<exec_depend>`
- No global mutable state in C++ nodes
- No placeholder `TODO` comments left in committed code
- No `using namespace std` in any header file
- No Python for geometry math that runs in a control loop

---

## README Standard (every project)

Every project README must include:

1. **One-sentence description** — what the robot does in simulation
2. **Architecture diagram** — ASCII node graph or image showing data flow
3. **Key technical decisions** — why C++ for X, why PyTorch for Y
4. **Dependencies** — exact ROS2 distro, apt packages, pip packages
5. **Build and launch instructions** — copy-pasteable commands
6. **Demo** — GIF or screenshot of RViz/Gazebo running
7. **Planned improvements** — shows engineering judgment to interviewers

---

## Portfolio Bar

Every project must demonstrate at least three of these to count as interview-ready:

- [ ] Multi-package ROS2 repo (C++ + Python)
- [ ] Learned / ML component (classifier, predictor, or policy)
- [ ] Explicit FSM or behavior tree
- [ ] Real-time C++ node (sensor callback + publisher)
- [ ] Parameterized config (no hardcoded values)
- [ ] Custom ROS2 message or service
- [ ] Gazebo simulation with sensor plugins
- [ ] Action server/client pattern
- [ ] Point cloud or depth image processing
- [ ] tf2 transform tree usage

---

## Code Quality Rules

- Write no comments by default. Add a comment only when the WHY is non-obvious (a hidden constraint, a workaround, a subtle invariant).
- Do not explain what the code does — names do that.
- No multi-line comment blocks. One short line max.
- No docstrings longer than one line for internal functions.
- Functions do one thing. If a function needs a comment to explain its sections, split it.
- Keep C++ node files under 300 lines. Split into multiple files if larger.
- Keep Python node files under 250 lines. Helper classes go in separate files.
