#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class TestPointCloudPublisher : public rclcpp::Node
{
public:
  TestPointCloudPublisher() : Node("test_pointcloud_publisher")
  {
    // Publishers
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_raw", 10);
    laserscan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    
    // Timer for publishing test data
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&TestPointCloudPublisher::publishTestData, this));
    
    RCLCPP_INFO(this->get_logger(), "Test PointCloud Publisher started");
  }

private:
  void publishTestData()
  {
    publishTestPointCloud();
    publishTestLaserScan();
  }
  
  void publishTestPointCloud()
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    // Create test point cloud - simple grid pattern
    cloud->width = 100;
    cloud->height = 100;
    cloud->is_dense = true;
    cloud->points.resize(cloud->width * cloud->height);
    
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      int x = i % cloud->width;
      int y = i / cloud->width;
      
      cloud->points[i].x = (x - 50) * 0.1; // -5m to 5m
      cloud->points[i].y = (y - 50) * 0.1; // -5m to 5m
      cloud->points[i].z = 0.1 + 0.01 * sin(x * 0.1) * cos(y * 0.1); // Small height variation
      cloud->points[i].intensity = 100 + 50 * sin(x * 0.2);
    }
    
    // Update width after adding points
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    
    // Add some distant points to test distance filtering
    for (int i = 0; i < 50; ++i) {
      pcl::PointXYZI point;
      point.x = 25.0 + i * 0.5;  // 25m to 50m range
      point.y = 0.0;
      point.z = 0.5;
      point.intensity = 200;
      cloud->points.push_back(point);
    }
    
    // Add some very close points to test minimum distance filtering
    for (int i = 0; i < 20; ++i) {
      pcl::PointXYZI point;
      point.x = 0.01 + i * 0.001;  // Very close points
      point.y = 0.0;
      point.z = 0.1;
      point.intensity = 50;
      cloud->points.push_back(point);
    }
    
    // Final update of dimensions
    cloud->width = cloud->points.size();
    cloud->height = 1;
    
    // Convert to ROS message
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "base_link";
    
    pointcloud_pub_->publish(cloud_msg);
  }
  
  void publishTestLaserScan()
  {
    auto scan = sensor_msgs::msg::LaserScan();
    
    scan.header.stamp = this->now();
    scan.header.frame_id = "base_link";
    
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = M_PI / 180.0; // 1 degree
    scan.time_increment = 0.0;
    scan.scan_time = 0.1;
    scan.range_min = 0.1;
    scan.range_max = 50.0;
    
    int num_readings = (scan.angle_max - scan.angle_min) / scan.angle_increment;
    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);
    
    // Generate test scan data
    for (int i = 0; i < num_readings; ++i) {
      double angle = scan.angle_min + i * scan.angle_increment;
      
      // Create some pattern - circular obstacle at 2m, wall at 5m
      double range;
      if (abs(angle) < M_PI/4) {
        range = 5.0; // Wall ahead
      } else if (abs(angle - M_PI/2) < M_PI/8 || abs(angle + M_PI/2) < M_PI/8) {
        range = 2.0; // Obstacles on sides
      } else {
        range = 10.0 + 5.0 * sin(angle * 3); // Varying distance
      }
      
      // Add some noise
      range += 0.1 * ((rand() % 100) / 100.0 - 0.5);
      
      // Add some very close and far points for testing
      if (i % 50 == 0) {
        range = 0.05; // Very close point
      } else if (i % 60 == 0) {
        range = 45.0; // Far point
      }
      
      scan.ranges[i] = range;
      scan.intensities[i] = 100 + 50 * sin(angle);
    }
    
    laserscan_pub_->publish(scan);
  }
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestPointCloudPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}