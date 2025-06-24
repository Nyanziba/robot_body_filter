/**
 * @file simple_robot_filter_demo.cpp
 * @brief Simplified demonstration of RobotBodyFilterProcessor
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pluginlib/class_loader.hpp>
#include <robot_body_filter/lidar_processor_base.h>

class SimpleRobotFilterDemo : public rclcpp::Node
{
public:
    SimpleRobotFilterDemo() : Node("simple_robot_filter_demo")
    {
        RCLCPP_INFO(this->get_logger(), "=== Simple Robot Body Filter Demo ===");
        
        // Initialize plugin loader
        plugin_loader_ = std::make_unique<pluginlib::ClassLoader<robot_body_filter::LidarProcessorBase>>(
            "robot_body_filter", "robot_body_filter::LidarProcessorBase");
        
        loadProcessor();
        
        RCLCPP_INFO(this->get_logger(), "Demo node initialized successfully!");
    }

private:
    void loadProcessor()
    {
        try {
            RCLCPP_INFO(this->get_logger(), "Loading RobotBodyFilterProcessor...");
            
            processor_ = plugin_loader_->createSharedInstance(
                "robot_body_filter/RobotBodyFilterProcessor");
            
            if (!processor_) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create processor instance");
                return;
            }
            
            // Initialize with this node
            if (!processor_->initialize(shared_from_this())) {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize processor");
                return;
            }
            
            // Configure the processor
            if (!processor_->configure()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to configure processor");
                return;
            }
            
            RCLCPP_INFO(this->get_logger(), "RobotBodyFilterProcessor loaded and configured successfully!");
            RCLCPP_INFO(this->get_logger(), "Processor name: %s", processor_->getProcessorName().c_str());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception loading processor: %s", e.what());
        }
    }

private:
    std::unique_ptr<pluginlib::ClassLoader<robot_body_filter::LidarProcessorBase>> plugin_loader_;
    std::shared_ptr<robot_body_filter::LidarProcessorBase> processor_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SimpleRobotFilterDemo>();
    
    RCLCPP_INFO(node->get_logger(), "Simple Robot Body Filter Demo completed successfully!");
    
    rclcpp::shutdown();
    return 0;
}