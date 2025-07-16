🤖 MoveIt2 Pick and Place Demo
A ROS2 demo showing advanced pick-and-place with MoveIt2 and the Panda robot arm.

📋 Assignment Submission - Free Task
Target Company: 株式会社キビテク
Assignment: Unique robot development program
Submission Type: Free task

🎯 Purpose（目的）
This project demonstrates core robotics programming skills by building a pick-and-place system using MoveIt2 and ROS2. It highlights:

Reliable motion planning

Error handling and recovery

Scalable ROS2 architecture

Realistic industrial robot behavior

⚙️ Specifications（仕様）
Core Tech Stack
ROS2: Humble

Planner: MoveIt2 + OMPL

Language: Python 3.8+

Robot: Franka Emika Panda

Sim: Gazebo

Build: colcon

Key Specs
Accuracy: ±1cm

Orientation: ±0.1 rad

Success Rate: >95%

Velocity Scaling: 10%

Workspace: 2m³

⭐ Appeal Points（アピールポイント）
Motion Planning
Uses position/orientation constraints and fallback strategies for high success.

Error Handling
Automatic retries, diagnostics, and safe degradation.

Modular Design
Clean code, easy to extend or integrate with perception.

Industrial Use Case
Conservative motion settings and precise target control.

ROS2 Best Practices
Action-based control, resource cleanup, async nodes.

📋 Prerequisites
System
Ubuntu 22.04 + ROS2 Humble

Python 3.8+, 4GB RAM+, 10GB storage

Install Packages
bash
Copy
Edit
sudo apt update
sudo apt install ros-humble-desktop-full ros-humble-moveit \
ros-humble-moveit-resources-panda-moveit-config \
ros-humble-moveit-visual-tools ros-humble-geometric-shapes \
python3-colcon-common-extensions python3-rosdep git
🚀 Build Instructions
1. Setup
bash
Copy
Edit
mkdir -p ~/moveit_project_ws/src
cd ~/moveit_project_ws/src
git clone <your-repository-url> simple_moveit_demo
sudo rosdep init
rosdep update
2. Dependencies
bash
Copy
Edit
cd ~/moveit_project_ws
rosdep install --from-paths src --ignore-src -r -y
3. Build
bash
Copy
Edit
colcon build --packages-select simple_moveit_demo --cmake-clean-cache
source install/setup.bash
ros2 pkg list | grep simple_moveit_demo
🎯 Operation Instructions
Terminal 1 - Start MoveIt2
bash
Copy
Edit
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
Terminal 2 - Run Demo
bash
Copy
Edit
ros2 run simple_moveit_demo pick_place_demo
Output
less
Copy
Edit
[INFO] [pick_place_demo]: Starting pick and place demo...
[INFO] [pick_place_demo]: Demo completed successfully!
🔧 Verification Methods
Basic Checks
bash
Copy
Edit
ros2 node list | grep pick_place_demo
ros2 topic echo /joint_states
ros2 action list | grep move_action
Repeated Runs
bash
Copy
Edit
for i in {1..5}; do
  ros2 run simple_moveit_demo pick_place_demo
  sleep 2
done
Visual Check
Use RViz2 to see motion

Watch for smooth path and collision avoidance

📊 Technical Configuration
yaml
Copy
Edit
Robot: Panda (7 DOF)
Planner: RRTConnect
Velocity: 0.1
Tolerance: 0.01m / 0.1rad

Pick:  [0.4, 0.1, 0.3]  
Place: [0.4, -0.1, 0.3]  
Orientation: [0.0, 0.707, 0.0, 0.707]
🔧 Customization Guide
Change Target
python
Copy
Edit
pose.pose.position.x = 0.5
pose.pose.position.y = 0.2
pose.pose.position.z = 0.4
Tweak Motion
python
Copy
Edit
move_group.set_planning_time(10.0)
move_group.set_max_velocity_scaling_factor(0.2)
🚨 Troubleshooting
No Action Server?

bash
Copy
Edit
ros2 action list | grep move_action
# Restart MoveIt2 if missing
Planning Fails?

bash
Copy
Edit
ros2 param get /move_group/panda_arm workspace
# Check if pose is out of bounds
Build Errors?

bash
Copy
Edit
rm -rf build/ install/ log/
colcon build --packages-select simple_moveit_demo
