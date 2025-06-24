#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <robot_body_filter/lidar_processor_base.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

/**
 * @brief Example node demonstrating how to use LiDAR processor plugins
 */
class LidarProcessorExampleNode : public rclcpp::Node, public std::enable_shared_from_this<LidarProcessorExampleNode>
{
public:
  LidarProcessorExampleNode() : Node("lidar_processor_example")
  {
    // Initialize the plugin loader
    plugin_loader_ = std::make_shared<pluginlib::ClassLoader<robot_body_filter::LidarProcessorBase>>(
        "robot_body_filter", "robot_body_filter::LidarProcessorBase");

    // Load and configure processors
    loadProcessors();

    // Create subscribers and publishers
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "points_raw", 10,
        std::bind(&LidarProcessorExampleNode::pointCloudCallback, this, std::placeholders::_1));

    laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&LidarProcessorExampleNode::laserScanCallback, this, std::placeholders::_1));

    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output_cloud", 10);
    laserscan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("output_scan", 10);

    RCLCPP_INFO(this->get_logger(), "LiDAR Processor Example Node initialized");
  }

private:
  void loadProcessors()
  {
    // Declare parameters for processor selection
    this->declare_parameter("active_processors", std::vector<std::string>{"robot_body_filter/DistanceFilterProcessor"});
    
    auto processor_names = this->get_parameter("active_processors").as_string_array();

    for (const auto& processor_name : processor_names) {
      try {
        RCLCPP_INFO(this->get_logger(), "Attempting to load processor: %s", processor_name.c_str());
        auto processor = plugin_loader_->createSharedInstance(processor_name);
        RCLCPP_INFO(this->get_logger(), "Plugin instantiated successfully: %s", processor_name.c_str());
        
        // Create a proper shared_ptr from the node
        auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
        if (processor->initialize(node_ptr)) {
          if (processor->configure()) {
            processors_.push_back(processor);
            RCLCPP_INFO(this->get_logger(), "Loaded processor: %s", processor->getProcessorName().c_str());
          } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure processor: %s", processor_name.c_str());
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to initialize processor: %s", processor_name.c_str());
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load processor %s: %s", processor_name.c_str(), e.what());
      }
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu processors", processors_.size());
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 processed_cloud = *msg;

    // Apply all processors in sequence
    for (auto& processor : processors_) {
      sensor_msgs::msg::PointCloud2 temp_cloud;
      if (processor->processPointCloud(std::make_shared<sensor_msgs::msg::PointCloud2>(processed_cloud), temp_cloud)) {
        processed_cloud = temp_cloud;
      } else {
        RCLCPP_WARN(this->get_logger(), "Processor %s failed to process point cloud", 
                    processor->getProcessorName().c_str());
      }
    }

    pointcloud_pub_->publish(processed_cloud);
  }

  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    sensor_msgs::msg::LaserScan processed_scan = *msg;

    // Apply all processors in sequence
    for (auto& processor : processors_) {
      sensor_msgs::msg::LaserScan temp_scan;
      if (processor->processLaserScan(std::make_shared<sensor_msgs::msg::LaserScan>(processed_scan), temp_scan)) {
        processed_scan = temp_scan;
      } else {
        RCLCPP_WARN(this->get_logger(), "Processor %s failed to process laser scan", 
                    processor->getProcessorName().c_str());
      }
    }

    laserscan_pub_->publish(processed_scan);
  }

  std::shared_ptr<pluginlib::ClassLoader<robot_body_filter::LidarProcessorBase>> plugin_loader_;
  std::vector<std::shared_ptr<robot_body_filter::LidarProcessorBase>> processors_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarProcessorExampleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}