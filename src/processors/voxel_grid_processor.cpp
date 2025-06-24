#include <robot_body_filter/processors/voxel_grid_processor.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.hpp>

namespace robot_body_filter
{

VoxelGridProcessor::VoxelGridProcessor()
  : leaf_size_x_(0.05)
  , leaf_size_y_(0.05)
  , leaf_size_z_(0.05)
{
  try {
    logger_ = rclcpp::get_logger("voxel_grid_processor");
  } catch (...) {
    // Fallback logger initialization
  }
}

bool VoxelGridProcessor::initialize(std::shared_ptr<rclcpp::Node> node)
{
  node_ = node;
  
  if (!node_) {
    RCLCPP_ERROR(logger_, "Node handle is null");
    return false;
  }
  
  RCLCPP_INFO(logger_, "Voxel grid processor initialized");
  return true;
}

bool VoxelGridProcessor::configure()
{
  if (!node_) {
    RCLCPP_ERROR(logger_, "Node not initialized");
    return false;
  }

  // Declare and get parameters
  node_->declare_parameter("voxel_grid.leaf_size_x", leaf_size_x_);
  node_->declare_parameter("voxel_grid.leaf_size_y", leaf_size_y_);
  node_->declare_parameter("voxel_grid.leaf_size_z", leaf_size_z_);

  leaf_size_x_ = node_->get_parameter("voxel_grid.leaf_size_x").as_double();
  leaf_size_y_ = node_->get_parameter("voxel_grid.leaf_size_y").as_double();
  leaf_size_z_ = node_->get_parameter("voxel_grid.leaf_size_z").as_double();

  // Configure PCL filters
  voxel_filter_.setLeafSize(leaf_size_x_, leaf_size_y_, leaf_size_z_);
  voxel_filter_i_.setLeafSize(leaf_size_x_, leaf_size_y_, leaf_size_z_);

  RCLCPP_INFO(logger_, "Voxel grid configured: leaf_size=[%.3f, %.3f, %.3f]", 
              leaf_size_x_, leaf_size_y_, leaf_size_z_);

  return true;
}

bool VoxelGridProcessor::processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input_cloud,
                                          sensor_msgs::msg::PointCloud2& output_cloud)
{
  if (!input_cloud) {
    RCLCPP_ERROR(logger_, "Input cloud is null");
    return false;
  }

  // Check if the cloud has intensity field
  bool has_intensity = false;
  for (const auto& field : input_cloud->fields) {
    if (field.name == "intensity") {
      has_intensity = true;
      break;
    }
  }

  if (has_intensity) {
    // Process with intensity
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZI>);
    
    pcl::fromROSMsg(*input_cloud, *pcl_input);
    applyVoxelGridFilter(*pcl_input, *pcl_output);
    pcl::toROSMsg(*pcl_output, output_cloud);
  } else {
    // Process without intensity
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromROSMsg(*input_cloud, *pcl_input);
    applyVoxelGridFilter(*pcl_input, *pcl_output);
    pcl::toROSMsg(*pcl_output, output_cloud);
  }

  output_cloud.header = input_cloud->header;
  return true;
}

bool VoxelGridProcessor::processLaserScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& input_scan,
                                         sensor_msgs::msg::LaserScan& output_scan)
{
  if (!input_scan) {
    RCLCPP_ERROR(logger_, "Input scan is null");
    return false;
  }

  // Convert laser scan to point cloud
  laser_geometry::LaserProjection projector;
  sensor_msgs::msg::PointCloud2 cloud;
  
  try {
    projector.projectLaser(*input_scan, cloud);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Failed to project laser scan: %s", e.what());
    return false;
  }

  // Apply voxel grid to point cloud
  sensor_msgs::msg::PointCloud2 filtered_cloud;
  if (!processPointCloud(std::make_shared<sensor_msgs::msg::PointCloud2>(cloud), filtered_cloud)) {
    return false;
  }

  // Convert back to laser scan (simplified approach)
  // Note: This is a simplified conversion. A more sophisticated approach would
  // properly reconstruct the scan from the downsampled points.
  output_scan = *input_scan;
  
  RCLCPP_WARN(logger_, "Voxel grid processing on laser scans is not fully implemented. "
                       "Consider using point cloud format for better results.");

  return true;
}

std::string VoxelGridProcessor::getProcessorName() const
{
  return "voxel_grid";
}

void VoxelGridProcessor::reset()
{
  // Reset any internal state if needed
  RCLCPP_INFO(logger_, "Voxel grid processor reset");
}

template<typename PointT>
void VoxelGridProcessor::applyVoxelGridFilter(const pcl::PointCloud<PointT>& input,
                                             pcl::PointCloud<PointT>& output)
{
  if constexpr (std::is_same_v<PointT, pcl::PointXYZ>) {
    voxel_filter_.setInputCloud(input.makeShared());
    voxel_filter_.filter(output);
  } else if constexpr (std::is_same_v<PointT, pcl::PointXYZI>) {
    voxel_filter_i_.setInputCloud(input.makeShared());
    voxel_filter_i_.filter(output);
  } else {
    // Fallback for other point types
    pcl::VoxelGrid<PointT> filter;
    filter.setLeafSize(leaf_size_x_, leaf_size_y_, leaf_size_z_);
    filter.setInputCloud(input.makeShared());
    filter.filter(output);
  }
}

// Explicit template instantiations
template void VoxelGridProcessor::applyVoxelGridFilter<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& output);
template void VoxelGridProcessor::applyVoxelGridFilter<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>& input, pcl::PointCloud<pcl::PointXYZI>& output);

}  // namespace robot_body_filter