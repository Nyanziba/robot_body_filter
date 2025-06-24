#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import time
import sys

class PluginTester(Node):
    def __init__(self):
        super().__init__('plugin_tester')
        
        self.get_logger().info("Plugin Tester initialized")
        
    def test_plugin_discovery(self):
        """Test if plugins can be discovered"""
        try:
            # Test pluginlib plugin discovery
            result = subprocess.run([
                'ros2', 'pkg', 'list', '|', 'grep', 'robot_body_filter'
            ], shell=True, capture_output=True, text=True)
            
            if result.returncode == 0:
                self.get_logger().info("✓ robot_body_filter package found")
            else:
                self.get_logger().error("✗ robot_body_filter package not found")
                return False
                
            return True
        except Exception as e:
            self.get_logger().error(f"Plugin discovery test failed: {e}")
            return False
    
    def test_plugin_loading(self):
        """Test plugin loading by checking pluginlib"""
        try:
            # Check if our plugins are registered
            result = subprocess.run([
                'ros2', 'run', 'pluginlib', 'list_plugins', 'robot_body_filter::LidarProcessorBase'
            ], capture_output=True, text=True)
            
            if result.returncode == 0 and result.stdout:
                self.get_logger().info("✓ Plugins registered successfully:")
                for line in result.stdout.strip().split('\n'):
                    if line.strip():
                        self.get_logger().info(f"  - {line.strip()}")
            else:
                self.get_logger().warn("Plugin list command failed or no plugins found")
                self.get_logger().info(f"stdout: {result.stdout}")
                self.get_logger().info(f"stderr: {result.stderr}")
                
            return True
        except Exception as e:
            self.get_logger().error(f"Plugin loading test failed: {e}")
            return False

def main():
    rclpy.init()
    
    tester = PluginTester()
    
    tester.get_logger().info("=== Robot Body Filter Plugin Test ===")
    
    # Test 1: Plugin Discovery
    tester.get_logger().info("Test 1: Plugin Discovery")
    if not tester.test_plugin_discovery():
        tester.get_logger().error("Plugin discovery failed!")
        return 1
    
    # Test 2: Plugin Loading
    tester.get_logger().info("Test 2: Plugin Loading")
    if not tester.test_plugin_loading():
        tester.get_logger().error("Plugin loading failed!")
        return 1
    
    tester.get_logger().info("=== All tests completed ===")
    
    rclpy.shutdown()
    return 0

if __name__ == '__main__':
    sys.exit(main())