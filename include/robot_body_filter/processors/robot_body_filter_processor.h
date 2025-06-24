#ifndef ROBOT_BODY_FILTER_ROBOT_BODY_FILTER_PROCESSOR_H
#define ROBOT_BODY_FILTER_ROBOT_BODY_FILTER_PROCESSOR_H

#include <robot_body_filter/lidar_processor_base.h>
#include <robot_body_filter/RobotBodyFilter.h>
#include <memory>

namespace robot_body_filter
{

/**
 * @brief Robot body filtering processor
 * 
 * This processor uses the existing RobotBodyFilter functionality to remove
 * the robot's own body from LiDAR data. It wraps the existing filter classes
 * in the plugin interface.
 */
class RobotBodyFilterProcessor : public LidarProcessorBase
{
public:
  /**
   * @brief Constructor
   */
  RobotBodyFilterProcessor();

  /**
   * @brief Destructor
   */
  virtual ~RobotBodyFilterProcessor() = default;

  /**
   * @brief Initialize the processor
   */
  bool initialize(std::shared_ptr<rclcpp::Node> node) override;

  /**
   * @brief Process PointCloud2 data
   */
  bool processPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input_cloud,
                        sensor_msgs::msg::PointCloud2& output_cloud) override;

  /**
   * @brief Process LaserScan data
   */
  bool processLaserScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& input_scan,
                       sensor_msgs::msg::LaserScan& output_scan) override;

  /**
   * @brief Get processor name
   */
  std::string getProcessorName() const override;

  /**
   * @brief Configure the processor
   */
  bool configure() override;

  /**
   * @brief Reset processor state
   */
  void reset() override;

private:
  /// Node handle for filter instances
  std::shared_ptr<rclcpp::Node> node_;
  
  /// Point cloud filter instance
  std::unique_ptr<RobotBodyFilterPointCloud2> pointcloud_filter_;
  
  /// Laser scan filter instance
  std::unique_ptr<RobotBodyFilterLaserScan> laserscan_filter_;
  
  /// Flag to indicate if filters are configured
  bool filters_configured_;

  /**
   * @brief Configure the point cloud filter
   */
  bool configurePointCloudFilter();

  /**
   * @brief Configure the laser scan filter
   */
  bool configureLaserScanFilter();
};

}  // namespace robot_body_filter

#endif  // ROBOT_BODY_FILTER_ROBOT_BODY_FILTER_PROCESSOR_H