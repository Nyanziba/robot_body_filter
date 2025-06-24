#include <pluginlib/class_list_macros.hpp>
#include <robot_body_filter/lidar_processor_base.h>
#include <robot_body_filter/processors/distance_filter_processor.h>
#include <robot_body_filter/processors/voxel_grid_processor.h>
#include <robot_body_filter/processors/robot_body_filter_processor.h>

// プラグイン登録
PLUGINLIB_EXPORT_CLASS(robot_body_filter::DistanceFilterProcessor, robot_body_filter::LidarProcessorBase)
PLUGINLIB_EXPORT_CLASS(robot_body_filter::VoxelGridProcessor, robot_body_filter::LidarProcessorBase)
PLUGINLIB_EXPORT_CLASS(robot_body_filter::RobotBodyFilterProcessor, robot_body_filter::LidarProcessorBase)