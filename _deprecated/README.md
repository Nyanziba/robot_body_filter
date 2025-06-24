# Robot Body Filter Examples

This directory contains examples and configurations for the robot_body_filter package.

## Directory Structure

### configs/
Configuration files for various robots and scenarios:
- `plugin_config.yaml` - Plugin system configuration for LiDAR processors
- `processor_params.yaml` - Parameter file for processors
- `full_example.yaml` - Complete filter chain configuration
- `jasmine_velodyne.yaml` - Jasmine robot with Velodyne LiDAR
- `tradr_*.yaml` - TRADR robot configurations
- `vlp16_params.yaml` - VLP-16 LiDAR parameters

### cpp_examples/
C++ example programs:
- `lidar_processor_example.cpp` - Main plugin system example
- `robot_body_filter_tutorial.cpp` - **Robot body filter tutorial with full_example.urdf**
- `test_pointcloud_publisher.cpp` - Test data generator
- `test_plugin_load.cpp` - Plugin loading test
- `debug_plugin_load.cpp` - Debug plugin loading
- `test_simple_plugin.cpp` - Simple plugin test

### launch_files/
ROS launch files:
- `body_filter_node.launch*` - Node launching configurations
- `full_example.launch` - Complete example launch

### meshes/
3D models and URDF files:
- `box.dae` - Test box mesh
- `full_example.urdf` - Example robot URDF

## Usage

### Robot Body Filter Tutorial (Recommended)
```bash
# Complete tutorial with NIFTi robot and RViz visualization
ros2 launch robot_body_filter robot_body_filter_tutorial.launch.py

# Tutorial without RViz
ros2 launch robot_body_filter robot_body_filter_tutorial.launch.py use_rviz:=false
```

### Plugin System Example
```bash
# Run with default configuration
ros2 run robot_body_filter lidar_processor_example

# Run with custom config
ros2 run robot_body_filter lidar_processor_example \
  --ros-args --params-file examples/configs/plugin_config.yaml

# Launch complete test
ros2 launch robot_body_filter test_plugins.launch.py
```

### Test Data Generation
```bash
# Generate test point clouds
ros2 run robot_body_filter test_pointcloud_publisher
```

## Plugin Configuration

The plugin system supports three main processors:

1. **DistanceFilterProcessor** - Distance-based filtering
2. **VoxelGridProcessor** - Voxel grid downsampling  
3. **RobotBodyFilterProcessor** - Robot body removal

See `configs/plugin_config.yaml` for configuration details.