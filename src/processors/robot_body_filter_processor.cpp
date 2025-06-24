#include <robot_body_filter/processors/robot_body_filter_processor.h>

namespace robot_body_filter
{

RobotBodyFilterProcessor::RobotBodyFilterProcessor()
  : filters_configured_(false)
{
  try {
    logger_ = rclcpp::get_logger("robot_body_filter_processor");
  } catch (...) {
    // Fallback logger initialization
  }
}

bool RobotBodyFilterProcessor::initialize(std::shared_ptr<rclcpp::Node> node)
{
  node_ = node;
  
  if (!node_) {
    RCLCPP_ERROR(logger_, "Node handle is null");
    return false;
  }

  RCLCPP_INFO(logger_, "Robot body filter processor initialized");
  return true;
}

bool RobotBodyFilterProcessor::configure()
{
  if (!node_) {
    RCLCPP_ERROR(logger_, "Node not initialized");
    return false;
  }

  // Create only PointCloud filter instance with the node handle
  pointcloud_filter_ = std::make_unique<RobotBodyFilterPointCloud2>(node_);

  if (!pointcloud_filter_) {
    RCLCPP_ERROR(logger_, "Point cloud filter instance not created");
    return false;
  }

  bool success = configurePointCloudFilter();
   
  filters_configured_ = success;
   
  if (success) {
    RCLCPP_INFO(logger_, "Robot body filter processor configured successfully");
  } else {
    RCLCPP_ERROR(logger_, "Failed to configure robot body filter processor");
  }
   
  return success;
}

bool RobotBodyFilterProcessor::configurePointCloudFilter()
{
  try {
    // The filter expects to be configured with node parameters
    // We use the existing filter's parameter handling
    if (pointcloud_filter_) {
      pointcloud_filter_->DeclareParameters();
      
      if (!pointcloud_filter_->configure()) {
        RCLCPP_ERROR(logger_, "Failed to configure point cloud filter");
        return false;
      }
    }
    
    RCLCPP_INFO(logger_, "Point cloud filter configured");
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception during point cloud filter configuration: %s", e.what());
    return false;
  }
}

bool RobotBodyFilterProcessor::processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input_cloud,
                                                 sensor_msgs::msg::PointCloud2& output_cloud)
{
  if (!filters_configured_) {
    RCLCPP_ERROR(logger_, "Filters not configured");
    return false;
  }

  if (!input_cloud) {
    RCLCPP_ERROR(logger_, "Input cloud is null");
    return false;
  }

  if (!pointcloud_filter_) {
    RCLCPP_ERROR(logger_, "Point cloud filter not initialized");
    return false;
  }

  try {
    // Use the existing filter's update method
    return pointcloud_filter_->update(*input_cloud, output_cloud);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception during point cloud processing: %s", e.what());
    return false;
  }
}

bool RobotBodyFilterProcessor::processLaserScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& input_scan,
                                                sensor_msgs::msg::LaserScan& output_scan)
{
  if (!filters_configured_) {
    RCLCPP_ERROR(logger_, "Filters not configured");
    return false;
  }

  if (!input_scan) {
    RCLCPP_ERROR(logger_, "Input scan is null");
    return false;
  }

  // LaserScan filter disabled in this build to avoid parameter duplication

  return true;
}

std::string RobotBodyFilterProcessor::getProcessorName() const
{
  return "robot_body_filter";
}

void RobotBodyFilterProcessor::reset()
{
  // Reset filters if needed
  if (pointcloud_filter_) {
    // The existing filter doesn't have a reset method, so we might need to recreate it
    RCLCPP_INFO(logger_, "Resetting point cloud filter");
  }
  
  filters_configured_ = false;
  RCLCPP_INFO(logger_, "Robot body filter processor reset");
}

}  // namespace robot_body_filter