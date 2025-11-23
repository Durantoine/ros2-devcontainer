# ROS2 Development Environment

Docker-based ROS2 Humble for Mac M3 with Gazebo and Foxglove.

## Requirements

- Docker Desktop
- [Foxglove Studio](https://foxglove.dev/download)

## Quick Start

```bash
docker-compose up -d
```

## Usage

### Gazebo + TurtleBot3

```bash
docker exec -it ros2_dev bash
export TURTLEBOT3_MODEL=burger
ros2 launch /workspaces/turtlebot3_fixed.launch.py use_sim_time:=true
```

### Foxglove Bridge

```bash
docker exec -it ros2_dev bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### Visualization

1. Open Foxglove Studio
2. Connect to `ws://localhost:8765`
3. Add 3D panel, set Fixed Frame to `odom`

### Control Robot

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.5}}" --rate 10
```

To stop the robot:
```bash
# In container
pkill -f 'ros2 topic pub /cmd_vel'
```

## Workspace Management

### Current Workspaces

```
workspaces/
├── TP3/                    # Robot status publisher/subscriber
├── TP4/                    # Empty workspace for new projects
└── turtlebot3_fixed.launch.py
```

### Switch Workspace

Edit `docker-compose.yml`:

```yaml
volumes:
  - ./workspaces:/workspaces
```

Change directory path, then:

```bash
docker-compose restart
```

### Build a Workspace

```bash
docker exec -it ros2_dev bash
cd /workspaces/TP3  # or TP4
colcon build
source install/setup.bash
```

### Create New Package

```bash
cd /workspaces/TP4
mkdir -p src && cd src

# Python package
ros2 pkg create --build-type ament_python my_package

# C++ package
ros2 pkg create --build-type ament_cmake my_package_cpp

# With dependencies
ros2 pkg create --build-type ament_python my_package \
  --dependencies rclpy std_msgs sensor_msgs
```

## ROS2 Commands Reference

### Build Commands

```bash
# Full build
colcon build --symlink-install

# Build specific package
colcon build --packages-select my_package

# Clean build
colcon build --symlink-install --cmake-clean-cache

# Parallel build
colcon build --parallel-workers 4
```

### Run Commands

```bash
# Run a node
ros2 run my_package my_node

# Launch file
ros2 launch my_package my_launch.py

# List nodes
ros2 node list

# Node info
ros2 node info /my_node
```

### Topics

```bash
# List topics
ros2 topic list

# Echo topic
ros2 topic echo /my_topic

# Publish to topic
ros2 topic pub /my_topic std_msgs/msg/String "data: 'Hello'"

# Topic info
ros2 topic info /my_topic

# Topic frequency
ros2 topic hz /my_topic
```

### Services

```bash
# List services
ros2 service list

# Call service
ros2 service call /my_service std_srvs/srv/Trigger

# Service type
ros2 service type /my_service
```

### Debug & Logs

```bash
# Run with debug logs
ros2 run my_package my_node --ros-args --log-level debug

# Echo rosout
ros2 topic echo /rosout

# System info
ros2 wtf

# Parameters
ros2 param list
ros2 param get /my_node my_param
```

## Architecture

```
Mac M3 (GPU)              Container (CPU)
┌─────────────────┐      ┌──────────────────┐
│ Foxglove Studio │◄─────┤ Foxglove Bridge  │
│ (WebGL)         │ :8765│                  │
└─────────────────┘      │ Gazebo (headless)│
                         └──────────────────┘
```

## Files

- `Dockerfile` - ROS2 Humble + Gazebo + TurtleBot3
- `docker-compose.yml` - Container config
- `.devcontainer/` - VSCode devcontainer
- `workspaces/turtlebot3_fixed.launch.py` - TurtleBot3 launch with fixed frame_prefix
- `workspaces/TP3/` - Robot status publisher/subscriber project
- `workspaces/TP4/` - Empty workspace for new projects

## Notes

### Frame Prefix Fix

Official turtlebot3_gazebo has bug with `frame_prefix: '/'` causing TF frames to have leading slashes (e.g., `/base_footprint` instead of `base_footprint`). This breaks Foxglove visualization.

**Solution**: Custom launch file `turtlebot3_fixed.launch.py` with `frame_prefix: ''`

### GPU Usage

- **Gazebo**: CPU only (runs in Docker container)
- **Foxglove Studio**: Mac GPU via WebGL (runs natively on Mac)

This architecture gives you GPU-accelerated 3D visualization in Foxglove while keeping Gazebo headless in the container.

## Troubleshooting

### Robot won't move

Check for duplicate `robot_state_publisher` processes:
```bash
docker exec -it ros2_dev bash
ps aux | grep robot_state_publisher
# Kill duplicates if found
pkill -f robot_state_publisher
```

### TF frame warnings in Foxglove

Make sure you're using `turtlebot3_fixed.launch.py` instead of the official `turtlebot3_gazebo` launch files.

### Foxglove can't connect

1. Check Foxglove Bridge is running:
   ```bash
   docker exec -it ros2_dev bash
   ros2 topic list | grep foxglove
   ```

2. Verify port 8765 is forwarded:
   ```bash
   docker ps
   ```

3. Reconnect in Foxglove Studio to `ws://localhost:8765`

### Clean build artifacts

```bash
docker exec -it ros2_dev bash
cd /workspaces/TP3  # or your workspace
rm -rf build install log
colcon build
```