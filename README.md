# ROS 2 Trajectory Tools Package

A ROS 2 package for recording, saving, and visualizing robot trajectories in multiple formats (JSON, CSV, YAML).

##  Package Overview

- **Trajectory Publisher**: Records robot path from `/odom` and provides service to save trajectories
- **Trajectory Reader**: Loads saved trajectories and visualizes them in RViz
- **Supported Formats**: JSON, CSV, and YAML

##  Installation

### Prerequisites
- ROS 2 Humble
- TurtleBot3 packages (tested on turtlebot3 you can use your own robot)

```bash
# Clone the repository
git clone https://github.com/PratikHandore/trajectory_tools.git
cd trajectory_tools

# Build the package
colcon build --packages-select trajectory_tools
source install/setup.bash
```

##  Dependencies

Ensure these are installed:
```bash
sudo apt install ros-humble-geometry-msgs ros-humble-visualization-msgs ros-humble-nav-msgs ros-humble-tf2-geometry-msgs
sudo apt install nlohmann-json3-dev libyaml-cpp-dev
```

##  Usage

### 1. Running the Nodes
```bash
# Terminal 1 - Publisher Node
ros2 run trajectory_tools trajectory_publisher_node

# Terminal 2 - Reader Node
ros2 run trajectory_tools trajectory_reader_node
```

### 2. Saving Trajectories
After moving your robot (or publishing to `/odom`):

```bash
# Save last 10 seconds as JSON
ros2 service call /save_trajectory trajectory_tools/srv/SaveTrajectory "{file_path: '/tmp/traj.json', format: 'json', duration: 10.0}"

# Save entire trajectory as CSV
ros2 service call /save_trajectory trajectory_tools/srv/SaveTrajectory "{file_path: '/tmp/traj.csv', format: 'csv', duration: 0.0}"
```

### 3. Loading Trajectories
```bash
ros2 service call /load_trajectory trajectory_tools/srv/LoadTrajectory "{file_path: '/tmp/traj.json', format: 'json'}"
```

### 4. Visualization in RViz
```bash
rviz2
```
Add these displays:
- MarkerArray (topic: `/trajectory_markers`) - Red = live path
- MarkerArray (topic: `/loaded_trajectory_markers`) - Green = loaded path

##  Service Definitions

### SaveTrajectory Service
```yaml
string file_path    # Output file path
string format       # "json", "csv", or "yaml"
float64 duration    # Save last N seconds (0 = save all)
---
bool success        # True if saved successfully
string message      # Status message
```

### LoadTrajectory Service
```yaml
string file_path    # Input file path
string format       # "json", "csv", or "yaml"
---
bool success        # True if loaded successfully
string message      # Status message
```

##  TurtleBot3 Integration

For use with TurtleBot3:
```bash
# Terminal 1 - Launch TurtleBot3
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2 - Launch TurtleBot3 nav2
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 3 - Teleoperation
ros2 run turtlebot3_teleop teleop_keyboard

# Terminal 4 - Trajectory Tools
ros2 run trajectory_tools trajectory_publisher_node
```
After running the nodes move the robot around. nodes will track the path.
##  Debugging Tips

Check available topics and services:
```bash
ros2 topic list
ros2 service list
```

View marker data:
```bash
ros2 topic echo /trajectory_markers
```

##  File Formats

### JSON Example
```json
{
  "trajectory": [
    {
      "header": {"frame_id": "odom"},
      "pose": {
        "position": {"x": 0.1, "y": 0.2, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
      }
    }
  ]
}
```

### CSV Example
```
frame_id,x,y,z,qx,qy,qz,qw
odom,0.1,0.2,0.0,0.0,0.0,0.0,1.0
```

