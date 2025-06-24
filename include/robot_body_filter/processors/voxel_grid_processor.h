#ifndef ROBOT_BODY_FILTER_VOXEL_GRID_PROCESSOR_H
#define ROBOT_BODY_FILTER_VOXEL_GRID_PROCESSOR_H

#include <robot_body_filter/lidar_processor_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace robot_body_filter
{

/**
 * @brief Voxel grid downsampling processor
 * 
 * This processor applies voxel grid downsampling to reduce the density of point clouds.
 * It's useful for reducing computational load while maintaining the general structure of the data.
 */
class VoxelGridProcessor : public LidarProcessorBase
{
public:
  /**
   * @brief Constructor
   */
  VoxelGridProcessor();

  /**
   * @brief Destructor
   */
  virtual ~VoxelGridProcessor() = default;

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
  /// Voxel size in X direction
  double leaf_size_x_;
  
  /// Voxel size in Y direction
  double leaf_size_y_;
  
  /// Voxel size in Z direction
  double leaf_size_z_;
  
  /// PCL voxel grid filter
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_i_;

  /**
   * @brief Apply voxel grid filtering to PCL point cloud
   */
  template<typename PointT>
  void applyVoxelGridFilter(const pcl::PointCloud<PointT>& input,
                           pcl::PointCloud<PointT>& output);
};

}  // namespace robot_body_filter

#endif  // ROBOT_BODY_FILTER_VOXEL_GRID_PROCESSOR_H