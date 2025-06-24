#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <robot_body_filter/lidar_processor_base.h>

int main(int argc, char** argv) {
    std::cout << "=== Plugin Loading Test ===" << std::endl;
    
    // Initialize ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("plugin_test_node");
    
    try {
        // Create plugin loader
        std::cout << "1. Creating plugin loader..." << std::endl;
        auto plugin_loader = std::make_shared<pluginlib::ClassLoader<robot_body_filter::LidarProcessorBase>>(
            "robot_body_filter", "robot_body_filter::LidarProcessorBase");
        
        // List available plugins
        std::cout << "2. Available plugins:" << std::endl;
        auto available_plugins = plugin_loader->getDeclaredClasses();
        for (const auto& plugin : available_plugins) {
            std::cout << "   - " << plugin << std::endl;
        }
        
        if (available_plugins.empty()) {
            std::cout << "   No plugins found! Check plugin registration." << std::endl;
            return 1;
        }
        
        // Test loading each plugin
        std::cout << "3. Testing plugin loading:" << std::endl;
        for (const auto& plugin_name : available_plugins) {
            try {
                std::cout << "   Loading " << plugin_name << "..." << std::endl;
                auto processor = plugin_loader->createSharedInstance(plugin_name);
                
                if (processor->initialize(node)) {
                    std::cout << "   ✓ " << processor->getProcessorName() << " initialized" << std::endl;
                    if (processor->configure()) {
                        std::cout << "   ✓ " << processor->getProcessorName() << " configured" << std::endl;
                    } else {
                        std::cout << "   ✗ " << processor->getProcessorName() << " configuration failed" << std::endl;
                    }
                } else {
                    std::cout << "   ✗ " << plugin_name << " initialization failed" << std::endl;
                }
            } catch (const std::exception& e) {
                std::cout << "   ✗ Error loading " << plugin_name << ": " << e.what() << std::endl;
            }
        }
        
        std::cout << "=== Plugin test completed ===" << std::endl;
        
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}