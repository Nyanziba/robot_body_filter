#ifndef ROBOT_BODY_FILTER_LIDAR_PROCESSOR_BASE_H
#define ROBOT_BODY_FILTER_LIDAR_PROCESSOR_BASE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>

namespace robot_body_filter
{

/**
 * @brief Base class for LiDAR data processing plugins
 * 
 * This class defines the interface that all LiDAR processing plugins must implement.
 * Plugins can process both PointCloud2 and LaserScan data types.
 */
class LidarProcessorBase
{
public:
  /**
   * @brief Virtual destructor
   */
  virtual ~LidarProcessorBase() = default;

  /**
   * @brief Initialize the plugin with ROS node handle and parameters
   * 
   * @param node Shared pointer to the ROS node
   * @return true if initialization was successful, false otherwise
   */
  virtual bool initialize(std::shared_ptr<rclcpp::Node> node) = 0;

  /**
   * @brief Process PointCloud2 data
   * 
   * @param input_cloud Input point cloud
   * @param output_cloud Processed output point cloud
   * @return true if processing was successful, false otherwise
   */
  virtual bool processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input_cloud,
                                sensor_msgs::msg::PointCloud2& output_cloud) = 0;

  /**
   * @brief Process LaserScan data
   * 
   * @param input_scan Input laser scan
   * @param output_scan Processed output laser scan
   * @return true if processing was successful, false otherwise
   */
  virtual bool processLaserScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& input_scan,
                               sensor_msgs::msg::LaserScan& output_scan) = 0;

  /**
   * @brief Get the name/type of this processor
   * 
   * @return String identifier for this processor
   */
  virtual std::string getProcessorName() const = 0;

  /**
   * @brief Configure the processor with parameters
   * 
   * This method is called after initialize() to set up processor-specific parameters.
   * 
   * @return true if configuration was successful, false otherwise
   */
  virtual bool configure() = 0;

  /**
   * @brief Reset the processor state
   * 
   * This method can be used to reset any internal state or clear caches.
   */
  virtual void reset() = 0;

protected:
  /// ROS node handle for parameter access and logging
  std::shared_ptr<rclcpp::Node> node_;
  
  /// Logger for this processor
  rclcpp::Logger logger_;

  /**
   * @brief Protected constructor to prevent direct instantiation
   */
  LidarProcessorBase() : logger_(rclcpp::get_logger("lidar_processor_base")) {}
};

}  // namespace robot_body_filter

#endif  // ROBOT_BODY_FILTER_LIDAR_PROCESSOR_BASE_H