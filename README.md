# 🤖 MoveIt2 Pick and Place Demo

A ROS2 pick and place demonstration using MoveIt2 and the Panda robot arm.

📄 Submission Overview (for 株式会社キビテク)
🧩 Purpose（目的）
Demonstrate basic pick-and-place robotics using MoveIt2 and a Panda arm, showcasing motion planning and ROS 2 integration for industrial automation simulation.

⚙️ Specifications（仕様）

ROS 2 Humble, MoveIt2, Python

Franka Emika Panda robot in simulation

Pick and place motion with precision constraints

Runs via ROS 2 launch and action clients

⭐ Appeal Points（アピールポイント）

MoveIt2 motion planning with constraints

Robust error handling and retries

Modular, customizable code

Ready for real-world extension (e.g., perception integration)

## 📋 Project Overview

This project demonstrates robotics programming capabilities through a pick and place system implementation. The demo showcases proficiency in:

- **Motion Planning**: Path planning using MoveIt2 algorithms
- **Constraint-Based Control**: Position and orientation constraints for reliable operation
- **Action-Based Architecture**: Asynchronous communication using ROS2 actions
- **Error Handling**: Comprehensive error management and recovery mechanisms

### 🎯 Project Specifications

- **Primary Function**: Automated pick and place operation using Franka Emika Panda robot arm
- **Target Application**: Industrial automation and research demonstrations
- **Performance**: Sub-centimeter precision with configurable safety parameters

## ⚙️ Technical Features

### 🔧 Implementation Features

1. **Motion Planning**
   - Multi-attempt planning with fallback strategies (10 attempts, 5s timeout)
   - Velocity/acceleration scaling for safe motion (10% scaling)
   - Workspace boundary enforcement for collision avoidance

2. **Precision Control**
   - Position constraints: 1cm tolerance box for precise positioning
   - Orientation constraints: 0.1 radian tolerance for consistent approach angles

3. **Error Handling**
   - Exception handling with detailed logging
   - Action server availability verification
   - Motion planning failure detection and reporting

## 📋 Prerequisites

- **ROS2 Humble**
- **MoveIt2**
- **Panda robot simulation packages**
- **Python 3.8+**
- **colcon**

```bash
# Install dependencies
sudo apt update
sudo apt install ros-humble-moveit ros-humble-moveit-resources-panda-moveit-config
sudo apt install ros-humble-moveit-visual-tools ros-humble-geometric-shapes
```

## 🚀 Build Instructions

```bash
# Create workspace
mkdir -p ~/moveit_project_ws/src
cd ~/moveit_project_ws/src

# Clone repository
git clone <your-repo-url> simple_moveit_demo

# Build
cd ~/moveit_project_ws
colcon build --packages-select simple_moveit_demo
source install/setup.bash
```

## 🎯 Operation Instructions

### Running the Demo

**Terminal 1 - Launch MoveIt:**
```bash
cd ~/moveit_project_ws
source install/setup.bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

**Terminal 2 - Run Demo:**
```bash
cd ~/moveit_project_ws
source install/setup.bash
ros2 run simple_moveit_demo pick_place_demo
```

### Expected Output

```
[INFO] [pick_place_demo]: MoveGroup action server found!
[INFO] [pick_place_demo]: Starting pick and place demo...
[INFO] [pick_place_demo]: Moving to pick position...
[INFO] [pick_place_demo]: pick motion completed successfully!
[INFO] [pick_place_demo]: Moving to place position...
[INFO] [pick_place_demo]: place motion completed successfully!
[INFO] [pick_place_demo]: Pick and place demo completed!
```

## 📊 Technical Specifications

### Configuration
- **Robot**: Franka Emika Panda (7-DOF)
- **Planning Group**: `panda_arm`
- **End Effector**: `panda_hand`
- **Planning Attempts**: 10
- **Planning Time**: 5.0s
- **Velocity/Acceleration Scaling**: 10%

### Precision
- **Position Tolerance**: ±1cm
- **Orientation Tolerance**: ±0.1 radians
- **Workspace**: -1.0m to +1.0m (cubic)

### Positions
- **Pick**: (0.4, 0.1, 0.3)
- **Place**: (0.4, -0.1, 0.3)

## 🔧 Customization

```python
# Modify positions in pick_place_demo.py
pick_pose.pose.position = Point(x=0.5, y=0.2, z=0.4)
pick_pose.pose.orientation = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)
```

## 🚨 Troubleshooting

```bash
# Verify MoveIt is running
ros2 action list | grep move_action

# Clean build if needed
colcon build --packages-select simple_moveit_demo --cmake-clean-cache
```
