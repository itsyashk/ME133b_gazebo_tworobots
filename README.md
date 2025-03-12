# Mars Exploration Simulation with Two-Robot SLAM in ROS2 Humble

This project creates a simulation environment that mimics a Mars-like landscape in Gazebo, featuring two differential-drive robots equipped with LiDAR sensors for Simultaneous Localization and Mapping (SLAM). The robots can be independently controlled via separate keyboard inputs, with one using arrow keys and the other using WASD.

## Overview

- **Mars Environment**: A Gazebo world that simulates Mars terrain with appropriate gravity and visual characteristics
- **Two Robots**: Differential-drive robots with LiDAR sensors for mapping
- **SLAM Processing**: Independent SLAM instances for each robot
- **Teleoperation**: Keyboard control for both robots
- **Visualization**: RViz2 for visualizing maps, sensor data, and TF frames

## Prerequisites

- Ubuntu 22.04 under WSL2
- ROS2 Humble
- Gazebo 11
- SLAM Toolbox
- Teleop Twist Keyboard package

## Building the Workspace

In a WSL terminal, run the following commands:

```bash
# Navigate to the workspace
cd ~/tests/133b_3-12

# Source ROS2
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Running the Simulation

The simulation requires multiple terminal windows to run different components.

### Terminal 1: Launch the Main Simulation

```bash
# Open WSL (if in Windows PowerShell)
wsl

# Navigate to workspace
cd ~/tests/133b_3-12

# Source ROS2
source /opt/ros/humble/setup.bash

# Source the workspace
source install/setup.bash

# Kill any existing Gazebo processes (if needed)
killall -9 gzserver gzclient

# Launch the simulation
ros2 launch mars_simulation mars_exploration.launch.py
```

This will start:
- Gazebo with the Mars world
- Two robots with LiDAR sensors
- SLAM processes for both robots
- RViz2 for visualization

### Terminal 2: Control Robot 1 (Arrow Keys)

```bash
# Open WSL (if in Windows PowerShell)
wsl

# Navigate to workspace
cd ~/tests/133b_3-12

# Source ROS2
source /opt/ros/humble/setup.bash

# Source the workspace
source install/setup.bash

# Launch the teleop node for robot 1
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel
```

### Terminal 3: Control Robot 2 (WASD Keys)

```bash
# Open WSL (if in Windows PowerShell)
wsl

# Navigate to workspace
cd ~/tests/133b_3-12

# Source ROS2
source /opt/ros/humble/setup.bash

# Source the workspace
source install/setup.bash

# Launch the teleop node for robot 2
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot2/cmd_vel
```

## Robot Control Instructions

### For Both Robots

The teleop_twist_keyboard provides the following controls:

```
Reading from the keyboard and publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
```

**Important**: Focus on the appropriate terminal when controlling each robot.

## Monitoring and Debugging

### List Available Topics

```bash
cd ~/tests/133b_3-12
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic list
```

### Check TF Frames

```bash
ros2 run tf2_ros tf2_monitor robot1/base_footprint robot1/map
```

### Inspect the Maps

```bash
ros2 topic echo /robot1/map/metadata
ros2 topic echo /robot2/map/metadata
```

## Troubleshooting

### Gazebo Already Running

If you encounter an error about Gazebo already running:

```
[Err] [Master.cc:96] EXCEPTION: Unable to start server[bind: Address already in use]. There is probably another Gazebo process running.
```

Run the following command before relaunching:

```bash
killall -9 gzserver gzclient
```

### PowerShell Command Issues

If you're in Windows PowerShell, remember that you need to execute one command at a time or use a WSL terminal directly. PowerShell doesn't support the `&&` chain operator with WSL commands.

### Missing SLAM Map

If you don't see a map in RViz:
1. Make sure the robots are moving
2. Check if the TF frames are working correctly
3. Try restarting the SLAM processes

## Project Structure

- **mars_world**: Contains the Gazebo world files
- **mars_robots**: Contains robot models (URDF/Xacro)
- **mars_simulation**: Contains launch files and configurations

## Additional Information

- The simulation uses two different colors for the robots to distinguish them.
- Each robot builds its own map independently.
- The SLAM parameters are set to work optimally with the LiDAR sensor range. 