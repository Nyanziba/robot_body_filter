#!/bin/bash

echo "=== Quick Plugin Test ==="
echo "Current directory: $(pwd)"
echo ""

# Source environments
echo "1. Sourcing environments..."
source /opt/ros/humble/setup.bash
echo "   ✓ ROS2 sourced"

cd ../../
echo "   Current directory: $(pwd)"
source install/setup.bash
echo "   ✓ Workspace sourced"

echo ""
echo "2. Environment check:"
echo "   AMENT_PREFIX_PATH contains workspace:"
echo $AMENT_PREFIX_PATH | tr ':' '\n' | grep robot_body_filter
echo ""

echo "3. Package check:"
ros2 pkg list | grep robot_body_filter
echo ""

echo "4. Plugin test (single processor):"
timeout 3 ros2 run robot_body_filter lidar_processor_example \
  --ros-args \
  -p active_processors:='["robot_body_filter/DistanceFilterProcessor"]' \
  -p distance_filter.min_distance:=0.1 \
  -p distance_filter.max_distance:=30.0 \
  2>&1 | head -10

echo ""
echo "=== Test completed ==="