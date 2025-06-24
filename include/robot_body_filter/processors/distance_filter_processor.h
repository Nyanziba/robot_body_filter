#ifndef ROBOT_BODY_FILTER_DISTANCE_FILTER_PROCESSOR_H
#define ROBOT_BODY_FILTER_DISTANCE_FILTER_PROCESSOR_H

#include <robot_body_filter/lidar_processor_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

namespace robot_body_filter
{

/**
 * @brief Distance-based filtering processor
 * 
 * This processor filters points based on their distance from the sensor origin.
 * Points closer than min_distance or farther than max_distance are removed.
 */
class DistanceFilterProcessor : public LidarProcessorBase
{
public:
  /**
   * @brief Constructor
   */
  DistanceFilterProcessor();

  /**
   * @brief Destructor
   */
  virtual ~DistanceFilterProcessor() = default;

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
  /// Minimum distance threshold
  double min_distance_;
  
  /// Maximum distance threshold
  double max_distance_;
  
  /// Whether to keep organized structure for point clouds
  bool keep_organized_;

  /**
   * @brief Filter points based on distance for PCL point cloud
   */
  template<typename PointT>
  void filterPointsByDistance(const pcl::PointCloud<PointT>& input,
                             pcl::PointCloud<PointT>& output);
};

}  // namespace robot_body_filter

#endif  // ROBOT_BODY_FILTER_DISTANCE_FILTER_PROCESSOR_H