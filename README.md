# Simple MoveIt Pick and Place Demo

A comprehensive ROS2 pick and place demonstration using MoveIt2 and the Panda robot arm. 

## Overview

This project implements a simple pick and place operation using the Franka Emika Panda robot arm in simulation. The demo showcases fundamental robotics concepts such as:

- Motion planning with MoveIt2
- Position and orientation constraints
- Workspace parameter configuration
- Action-based robot control
- ROS2 node development

The robot arm moves between two predefined positions, simulating a basic pick and place operation commonly used in industrial automation and research applications.

## Prerequisites

Before running this demo, ensure you have the following installed:

- **ROS2 Humble** (or compatible version)
- **MoveIt2** - Motion planning framework
- **Panda robot simulation packages** - `moveit_resources_panda_moveit_config`
- **Python 3.8+**
- **colcon** - ROS2 build tool

### Installing Dependencies

```bash
# Install MoveIt2 and Panda resources
sudo apt update
sudo apt install ros-humble-moveit ros-humble-moveit-resources-panda-moveit-config

# Install additional dependencies
sudo apt install ros-humble-moveit-visual-tools ros-humble-geometric-shapes
```

## Installation

1. **Create and navigate to workspace:**
```bash
mkdir -p ~/moveit_project_ws/src
cd ~/moveit_project_ws
```

2. **Clone this repository:**
```bash
cd src
git clone <your-repo-url> simple_moveit_demo
```

3. **Build the workspace:**
```bash
cd ~/moveit_project_ws
colcon build --packages-select simple_moveit_demo
```

4. **Source the workspace:**
```bash
source install/setup.bash
```

## Usage

### Running the Demo

1. **Start MoveIt demo in the first terminal:**
```bash
# Terminal 1
cd ~/moveit_project_ws
source install/setup.bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```

This launches:
- RViz with MoveIt motion planning plugin
- Panda robot model visualization
- MoveIt move_group action server

2. **Run the pick and place demo in a second terminal:**
```bash
# Terminal 2
cd ~/moveit_project_ws
source install/setup.bash
ros2 run simple_moveit_demo pick_place_demo
```

### Expected Output

When running successfully, you should see output similar to:
```
[INFO] [pick_place_demo]: Waiting for MoveGroup action server...
[INFO] [pick_place_demo]: MoveGroup action server found!
[INFO] [pick_place_demo]: Starting pick and place demo...
[INFO] [pick_place_demo]: Moving to pick position...
[INFO] [pick_place_demo]: pick goal accepted, waiting for result...
[INFO] [pick_place_demo]: pick motion completed successfully!
[INFO] [pick_place_demo]: Moving to place position...
[INFO] [pick_place_demo]: place goal accepted, waiting for result...
[INFO] [pick_place_demo]: place motion completed successfully!
[INFO] [pick_place_demo]: Pick and place demo completed!
```

## What it Does

The demonstration performs the following sequence:

1. **Initialization**: Creates a ROS2 node and connects to the MoveGroup action server
2. **Pick Motion**: Moves the robot end-effector to the pick position (0.4, 0.1, 0.3)
3. **Place Motion**: Moves the robot end-effector to the place position (0.4, -0.1, 0.3)
4. **Completion**: Reports successful completion of the demo

### Robot Positions

- **Pick Position**: 
  - X: 0.4m (forward from base)
  - Y: 0.1m (left from base)
  - Z: 0.3m (up from base)

- **Place Position**:
  - X: 0.4m (forward from base)  
  - Y: -0.1m (right from base)
  - Z: 0.3m (up from base)

Both positions maintain the same orientation (no rotation).

## Code Structure

```
moveit_project_ws/
├── src/
│   └── simple_moveit_demo/
│       ├── package.xml
│       ├── setup.py
│       ├── simple_moveit_demo/
│       │   ├── init.py
│       │   └── pick_place_demo.py
│       ├── launch/
│       │   ├── demo.launch.py
│       │   └── moveit.launch.py
│       ├── config/
│       │   ├── panda_arm.yaml
│       │   └── controllers.yaml
│       └── urdf/
│           └── panda_arm.urdf.xacro
```

### Key Components

**PickPlaceDemo Class**: Main node class that handles:
- MoveGroup action client initialization
- Motion planning requests
- Constraint setup (position and orientation)
- Workspace parameter configuration
- Error handling and logging

**Key Methods**:
- `run_demo()`: Orchestrates the pick and place sequence
- `move_to_pose()`: Handles individual motion planning and execution

## Technical Details

### Motion Planning Parameters

- **Planning Group**: `panda_arm`
- **End Effector**: `panda_hand`
- **Planning Attempts**: 10
- **Planning Time**: 5.0 seconds
- **Velocity Scaling**: 0.1 (10% of maximum)
- **Acceleration Scaling**: 0.1 (10% of maximum)

### Constraints

**Position Constraints**:
- Tolerance: 1cm box around target position
- Applied to `panda_hand` link

**Orientation Constraints**:
- Tolerance: 0.1 radians on all axes
- Maintains consistent end-effector orientation

### Workspace Parameters

- **Frame**: `panda_link0` (robot base)
- **Bounds**: -1.0m to +1.0m in all directions

## Troubleshooting

### Common Issues

1. **"MoveGroup action server not found"**
   - Ensure MoveIt demo is running first
   - Check that `moveit_resources_panda_moveit_config` is installed
   - Verify ROS2 environment is properly sourced

2. **"Goal rejected" or planning failures**
   - Check if target positions are reachable
   - Verify robot model loads correctly in RViz
   - Increase planning time or attempts if needed

3. **Import errors**
   - Ensure all ROS2 and MoveIt2 packages are installed
   - Check Python dependencies
   - Rebuild workspace with `colcon build`

4. **RCL shutdown errors**
   - This is a known issue with rapid node shutdown
   - Does not affect functionality
   - Fixed in the current code version

### Debugging Commands

```bash
# Check if MoveGroup action server is running
ros2 action list

# Monitor robot joint states
ros2 topic echo /joint_states

# Check available planning groups
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{}"

# List active nodes
ros2 node list
```

### Modifying Positions

Edit the pose definitions in `pick_place_demo.py`:

```python
# Example: Change pick position
pick_pose.pose.position = Point(x=0.5, y=0.2, z=0.4)

# Example: Add rotation
pick_pose.pose.orientation = Quaternion(x=0.0, y=0.707, z=0.0, w=0.707)
```
