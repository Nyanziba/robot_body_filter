#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <robot_body_filter/RobotBodyFilter.h>
#include <pluginlib/class_loader.hpp>
#include <filters/filter_base.hpp>

namespace Oedo_robot_body_filter {
class Oedo : public rclcpp::Node
{
private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publishFilteredCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    pluginlib::ClassLoader<filters::FilterBase<sensor_msgs::msg::PointCloud2>> filter_loader_;
    std::shared_ptr<filters::FilterBase<sensor_msgs::msg::PointCloud2>> filter_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr inputSub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr outputPub;

public:
    Oedo(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~Oedo();
};
}

Oedo_robot_body_filter::Oedo::Oedo(const rclcpp::NodeOptions& options) 
    : rclcpp::Node("oedo", options),
      filter_loader_("filters", "filters::FilterBase<sensor_msgs::msg::PointCloud2>")
{
    try {
        // Create filter instance using pluginlib
        const std::string plugin_name = "robot_body_filter/RobotBodyFilterPointCloud2";
        filter_ = filter_loader_.createSharedInstance(plugin_name);
        
        if (!filter_) {
            throw std::runtime_error("Failed to create filter instance");
        }
        
        // Configure the filter with node parameter
        if (!filter_->configure("robot_body_filter", 
                               "robot_body_filter/RobotBodyFilterPointCloud2",
                               this->get_node_logging_interface(),
                               this->get_node_parameters_interface())) {
            throw std::runtime_error("Failed to configure filter");
        }
        
        // Set up publisher
        outputPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);
        
        // Set up subscriber
        inputSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "livox/lidar", 10,
            std::bind(&Oedo::pointCloudCallback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Robot body filter configured successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to configure robot body filter: %s", e.what());
        throw;
    }
}

Oedo_robot_body_filter::Oedo::~Oedo()
{
}

void Oedo_robot_body_filter::Oedo::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    sensor_msgs::msg::PointCloud2 filteredCloud;
    if (filter_->update(*msg, filteredCloud)) {
        auto filteredPtr = std::make_shared<sensor_msgs::msg::PointCloud2>(filteredCloud);
        publishFilteredCloud(filteredPtr);
    } else {
        RCLCPP_WARN(this->get_logger(), "Failed to filter point cloud");
    }
}

void Oedo_robot_body_filter::Oedo::publishFilteredCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    outputPub->publish(*msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(Oedo_robot_body_filter::Oedo)