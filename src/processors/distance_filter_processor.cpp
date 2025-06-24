#include <robot_body_filter/processors/distance_filter_processor.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <cmath>

namespace robot_body_filter
{

DistanceFilterProcessor::DistanceFilterProcessor()
  : min_distance_(0.1)
  , max_distance_(100.0)
  , keep_organized_(false)
{
  // Initialize logger in a safe way
  try {
    logger_ = rclcpp::get_logger("distance_filter_processor");
  } catch (...) {
    // Fallback logger initialization
  }
}

bool DistanceFilterProcessor::initialize(std::shared_ptr<rclcpp::Node> node)
{
  node_ = node;
  
  if (!node_) {
    RCLCPP_ERROR(logger_, "Node handle is null");
    return false;
  }
  
  RCLCPP_INFO(logger_, "Distance filter processor initialized");
  return true;
}

bool DistanceFilterProcessor::configure()
{
  if (!node_) {
    RCLCPP_ERROR(logger_, "Node not initialized");
    return false;
  }

  // Declare and get parameters
  node_->declare_parameter("distance_filter.min_distance", min_distance_);
  node_->declare_parameter("distance_filter.max_distance", max_distance_);
  node_->declare_parameter("distance_filter.keep_organized", keep_organized_);

  min_distance_ = node_->get_parameter("distance_filter.min_distance").as_double();
  max_distance_ = node_->get_parameter("distance_filter.max_distance").as_double();
  keep_organized_ = node_->get_parameter("distance_filter.keep_organized").as_bool();

  RCLCPP_INFO(logger_, "Distance filter configured: min=%.2f, max=%.2f, organized=%s", 
              min_distance_, max_distance_, keep_organized_ ? "true" : "false");

  return true;
}

bool DistanceFilterProcessor::processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input_cloud,
                                               sensor_msgs::msg::PointCloud2& output_cloud)
{
  if (!input_cloud) {
    RCLCPP_ERROR(logger_, "Input cloud is null");
    return false;
  }

  // Convert to PCL format
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZI>);
  
  pcl::fromROSMsg(*input_cloud, *pcl_input);

  // Apply distance filtering
  filterPointsByDistance(*pcl_input, *pcl_output);

  // Convert back to ROS format
  pcl::toROSMsg(*pcl_output, output_cloud);
  output_cloud.header = input_cloud->header;

  return true;
}

bool DistanceFilterProcessor::processLaserScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& input_scan,
                                              sensor_msgs::msg::LaserScan& output_scan)
{
  if (!input_scan) {
    RCLCPP_ERROR(logger_, "Input scan is null");
    return false;
  }

  output_scan = *input_scan;  // Copy header and metadata
  output_scan.ranges.clear();
  output_scan.intensities.clear();

  const bool has_intensities = !input_scan->intensities.empty();
  
  // Filter ranges based on distance
  for (size_t i = 0; i < input_scan->ranges.size(); ++i) {
    const float range = input_scan->ranges[i];
    
    // Check if range is valid and within distance bounds
    if (std::isfinite(range) && range >= min_distance_ && range <= max_distance_) {
      output_scan.ranges.push_back(range);
      if (has_intensities) {
        output_scan.intensities.push_back(input_scan->intensities[i]);
      }
    } else {
      // Replace with NaN for invalid ranges
      output_scan.ranges.push_back(std::numeric_limits<float>::quiet_NaN());
      if (has_intensities) {
        output_scan.intensities.push_back(0.0);
      }
    }
  }

  return true;
}

std::string DistanceFilterProcessor::getProcessorName() const
{
  return "distance_filter";
}

void DistanceFilterProcessor::reset()
{
  // Nothing to reset for this processor
  RCLCPP_INFO(logger_, "Distance filter processor reset");
}

template<typename PointT>
void DistanceFilterProcessor::filterPointsByDistance(const pcl::PointCloud<PointT>& input,
                                                    pcl::PointCloud<PointT>& output)
{
  output.header = input.header;
  output.clear();
  
  if (keep_organized_) {
    output.width = input.width;
    output.height = input.height;
    output.is_dense = false;
    output.resize(input.size());
    
    for (size_t i = 0; i < input.size(); ++i) {
      const auto& point = input.points[i];
      const double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      
      if (distance >= min_distance_ && distance <= max_distance_) {
        output.points[i] = point;
      } else {
        // Create invalid point
        PointT invalid_point;
        invalid_point.x = invalid_point.y = invalid_point.z = std::numeric_limits<float>::quiet_NaN();
        output.points[i] = invalid_point;
      }
    }
  } else {
    output.width = 0;
    output.height = 1;
    output.is_dense = true;
    
    for (const auto& point : input.points) {
      const double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      
      if (distance >= min_distance_ && distance <= max_distance_) {
        output.points.push_back(point);
      }
    }
    
    output.width = output.points.size();
  }
}

// Explicit template instantiations
template void DistanceFilterProcessor::filterPointsByDistance<pcl::PointXYZ>(
    const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& output);
template void DistanceFilterProcessor::filterPointsByDistance<pcl::PointXYZI>(
    const pcl::PointCloud<pcl::PointXYZI>& input, pcl::PointCloud<pcl::PointXYZI>& output);

}  // namespace robot_body_filter