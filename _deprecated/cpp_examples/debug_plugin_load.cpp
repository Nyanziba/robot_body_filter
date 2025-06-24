#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include <robot_body_filter/lidar_processor_base.h>

int main(int argc, char** argv) {
    std::cout << "=== Debug Plugin Loading ===" << std::endl;
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("debug_plugin_test");
    
    try {
        std::cout << "1. Creating ClassLoader..." << std::endl;
        pluginlib::ClassLoader<robot_body_filter::LidarProcessorBase> loader(
            "robot_body_filter", "robot_body_filter::LidarProcessorBase");
        
        std::cout << "2. Getting declared classes..." << std::endl;
        auto classes = loader.getDeclaredClasses();
        
        std::cout << "3. Found " << classes.size() << " classes:" << std::endl;
        for (const auto& cls : classes) {
            std::cout << "   - " << cls << std::endl;
            
            try {
                std::cout << "     Description: " << loader.getClassDescription(cls) << std::endl;
            } catch (const std::exception& e) {
                std::cout << "     Description error: " << e.what() << std::endl;
            }
            
            try {
                std::cout << "     Base class: " << loader.getBaseClassType() << std::endl;
            } catch (const std::exception& e) {
                std::cout << "     Base class error: " << e.what() << std::endl;
            }
        }
        
        if (classes.empty()) {
            std::cout << "4. No classes found. Debugging plugin discovery..." << std::endl;
            
            // Try to get available libraries
            try {
                auto libs = loader.getDeclaredClasses();
                std::cout << "   Available libraries: " << libs.size() << std::endl;
                for (const auto& lib : libs) {
                    std::cout << "     - " << lib << std::endl;
                }
            } catch (const std::exception& e) {
                std::cout << "   Library error: " << e.what() << std::endl;
            }
        } else {
            std::cout << "4. Testing plugin instantiation..." << std::endl;
            for (const auto& cls : classes) {
                try {
                    auto instance = loader.createSharedInstance(cls);
                    std::cout << "   ✓ " << cls << " instantiated successfully" << std::endl;
                } catch (const std::exception& e) {
                    std::cout << "   ✗ " << cls << " instantiation failed: " << e.what() << std::endl;
                }
            }
        }
        
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}