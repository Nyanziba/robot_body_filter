/**
 * @file robot_body_filter_tutorial.cpp
 * @brief Tutorial demonstrating RobotBodyFilterProcessor with full_example.urdf
 * 
 * This tutorial shows how to:
 * 1. Load a robot URDF model
 * 2. Configure the RobotBodyFilterProcessor
 * 3. Process LiDAR data to remove robot body parts
 * 4. Visualize the filtering results
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <pluginlib/class_loader.hpp>
#include <robot_body_filter/lidar_processor_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>
#include <random>

class RobotBodyFilterTutorial : public rclcpp::Node, public std::enable_shared_from_this<RobotBodyFilterTutorial>
{
public:
    RobotBodyFilterTutorial() : Node("robot_body_filter_tutorial")
    {
        try {

            

            RCLCPP_INFO(this->get_logger(), "=== Robot Body Filter Tutorial ===");
            RCLCPP_INFO(this->get_logger(), "This tutorial demonstrates filtering robot body from LiDAR data");
            
            // Initialize plugin loader
            plugin_loader_ = std::make_unique<pluginlib::ClassLoader<robot_body_filter::LidarProcessorBase>>(
                "robot_body_filter", "robot_body_filter::LidarProcessorBase");
            
            // 初期化処理を100ms遅延で実行
            init_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                [this]() {
                    this->loadProcessor();
                    this->setupRosInterfaces();
                    this->setupTfBroadcasters();

                    // Timer for publishing synthetic LiDAR data (after full init)
                    data_timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(100),
                        std::bind(&RobotBodyFilterTutorial::publishTestData, this));

                    // Timer for updating robot joint states (after tf broadcaster is ready)
                    joint_timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(50),
                        std::bind(&RobotBodyFilterTutorial::publishJointStates, this));

                    // 初回実行後タイマーを停止
                    if (init_timer_) {
                        init_timer_->cancel();
                    }
                }
            );
            
            RCLCPP_INFO(this->get_logger(), "Tutorial node initialized successfully!");
            RCLCPP_INFO(this->get_logger(), "Topics:");
            RCLCPP_INFO(this->get_logger(), "  Input:  /raw_pointcloud");
            RCLCPP_INFO(this->get_logger(), "  Output: /filtered_pointcloud");
            RCLCPP_INFO(this->get_logger(), "  Output: /debug_pointcloud");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in RobotBodyFilterTutorial constructor: %s", e.what());
            throw;
        }
    }

public:
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
            
            // Create a non-owning shared_ptr from this node to avoid shared_from_this issues
            auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
            
            // Initialize with this node
            if (!processor_->initialize(node_ptr)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize processor");
                return;
            }
            
            // Configure the processor
            if (!processor_->configure()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to configure processor");
                return;
            }
            
            RCLCPP_INFO(this->get_logger(), "RobotBodyFilterProcessor loaded and configured successfully!");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception loading processor: %s", e.what());
        }
    }
    
    void setupRosInterfaces()
    {
        // Subscribers
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/raw_pointcloud", 10,
            std::bind(&RobotBodyFilterTutorial::processPointCloud, this, std::placeholders::_1));
        
        // Publishers
        filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pointcloud", 10);
        debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug_pointcloud", 10);
        raw_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/raw_pointcloud", 10);
        removal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/removed_area_marker", 10);
        
        RCLCPP_INFO(this->get_logger(), "ROS interfaces setup complete");
    }
    
    void setupTfBroadcasters()
    {
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        // Publish static transform from base_link to laser_base
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = this->now();
        static_transform.header.frame_id = "base_link";
        static_transform.child_frame_id = "laser_base";
        static_transform.transform.translation.x = -1.5;
        static_transform.transform.translation.y = 0.0;
        static_transform.transform.translation.z = 0.0;
        static_transform.transform.rotation.w = 1.0;
        
        static_tf_broadcaster_->sendTransform(static_transform);
        
        laser_rotation_ = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "TF broadcasters setup complete");
    }
    
    void publishJointStates()
    {
        // Simulate rotating laser
        laser_rotation_ += 0.05;  // Rotate at ~1 rad/s
        if (laser_rotation_ > M_PI) laser_rotation_ = -M_PI;
        
        // Publish dynamic transform from laser_base to laser
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "laser_base";
        transform.child_frame_id = "laser";
        
        tf2::Quaternion q;
        q.setRPY(laser_rotation_, 0, 0);  // Rotate around X-axis
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(transform);
    }
    
    void publishTestData()
    {
        // Create synthetic point cloud data (random points in range -100~100)
        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.header.frame_id = "laser";
        cloud.header.stamp = pcl_conversions::toPCL(this->now());

        static std::mt19937 rng(std::random_device{}());
        std::uniform_real_distribution<double> dist(-3.0, 3.0);

        const size_t NUM_POINTS = 50000;  // adjustable
        cloud.points.reserve(NUM_POINTS);
        for (size_t i = 0; i < NUM_POINTS; ++i) {
            pcl::PointXYZ p;
            p.x = dist(rng);
            p.y = dist(rng);
            p.z = dist(rng);
            cloud.points.push_back(p);
        }

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = false;
        
        pcl::toROSMsg(cloud, *cloud_msg);
        cloud_msg->header.stamp = this->now();
        cloud_msg->header.frame_id = "laser";
        
        raw_pub_->publish(*cloud_msg);
        
        // Show statistics
        static int count = 0;
        if (++count % 50 == 0) {  // Every 5 seconds
            RCLCPP_INFO(this->get_logger(), 
                "Generated test cloud with %zu points from laser frame", 
                cloud.points.size());
        }
    }
    
    void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud)
    {
        if (!processor_) {
            RCLCPP_WARN(this->get_logger(), "Processor not initialized");
            return;
        }
        
        try {
            sensor_msgs::msg::PointCloud2 filtered_cloud;
            
            // Process the point cloud
            bool success = processor_->processPointCloud(input_cloud, filtered_cloud);
            
            if (success) {
                // Publish filtered result
                filtered_pub_->publish(filtered_cloud);
                
                // Calculate and log filtering statistics
                size_t input_points = input_cloud->width * input_cloud->height;
                size_t output_points = filtered_cloud.width * filtered_cloud.height;
                double filter_ratio = (double)(input_points - output_points) / input_points * 100.0;
                
                static int log_count = 0;
                if (++log_count % 20 == 0) {  // Log every 2 seconds
                    RCLCPP_INFO(this->get_logger(), 
                        "Filtering: %zu -> %zu points (%.1f%% removed)",
                        input_points, output_points, filter_ratio);
                }
                
                // Create debug visualization
                publishDebugVisualization(input_cloud, filtered_cloud);
                
            } else {
                RCLCPP_WARN(this->get_logger(), "Point cloud processing failed");
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in point cloud processing: %s", e.what());
        }
    }
    
    void publishDebugVisualization(
        const sensor_msgs::msg::PointCloud2::SharedPtr input,
        const sensor_msgs::msg::PointCloud2& filtered)
    {
        // Create a colored point cloud showing filtered vs original points
        pcl::PointCloud<pcl::PointXYZRGB> debug_cloud;
        
        // Convert input to PCL
        pcl::PointCloud<pcl::PointXYZ> input_pcl;
        pcl::fromROSMsg(*input, input_pcl);
        
        // Convert filtered to PCL  
        pcl::PointCloud<pcl::PointXYZ> filtered_pcl;
        pcl::fromROSMsg(filtered, filtered_pcl);
        
        // Add filtered points in green
        for (const auto& point : filtered_pcl.points) {
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;
            colored_point.r = 0;
            colored_point.g = 255;
            colored_point.b = 0;
            debug_cloud.points.push_back(colored_point);
        }
        
        debug_cloud.width = debug_cloud.points.size();
        debug_cloud.height = 1;
        debug_cloud.is_dense = true;
        debug_cloud.header = filtered_pcl.header;
        
        sensor_msgs::msg::PointCloud2 debug_msg;
        pcl::toROSMsg(debug_cloud, debug_msg);
        debug_msg.header.stamp = this->now();
        debug_msg.header.frame_id = filtered.header.frame_id;
        
        debug_pub_->publish(debug_msg);

        // Publish bounding box marker for removed points
        // Build a hash set of kept points for quick lookup
        std::unordered_set<std::string> kept_hash;
        kept_hash.reserve(filtered_pcl.points.size());
        auto hash_point = [](const pcl::PointXYZ& p){
            std::ostringstream ss;
            ss << std::round(p.x*1000) << "," << std::round(p.y*1000) << "," << std::round(p.z*1000);
            return ss.str();
        };
        for (const auto& p : filtered_pcl.points) {
            kept_hash.insert(hash_point(p));
        }
        bool has_removed = false;
        double min_x=std::numeric_limits<double>::max(), min_y=min_x, min_z=min_x;
        double max_x=-min_x, max_y=max_x, max_z=max_x;
        for (const auto& p : input_pcl.points) {
            if (kept_hash.find(hash_point(p)) == kept_hash.end()) {
                has_removed = true;
                min_x = std::min(min_x, (double)p.x);
                min_y = std::min(min_y, (double)p.y);
                min_z = std::min(min_z, (double)p.z);
                max_x = std::max(max_x, (double)p.x);
                max_y = std::max(max_y, (double)p.y);
                max_z = std::max(max_z, (double)p.z);
            }
        }
        if (has_removed) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = input->header.frame_id;
            marker.header.stamp = this->now();
            marker.ns = "removed_area";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = (min_x + max_x) / 2.0;
            marker.pose.position.y = (min_y + max_y) / 2.0;
            marker.pose.position.z = (min_z + max_z) / 2.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = std::max(0.001, max_x - min_x);
            marker.scale.y = std::max(0.001, max_y - min_y);
            marker.scale.z = std::max(0.001, max_z - min_z);
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.3f;  // semi-transparent red
            removal_marker_pub_->publish(marker);
        }
    }

private:
    // Plugin management
    std::unique_ptr<pluginlib::ClassLoader<robot_body_filter::LidarProcessorBase>> plugin_loader_;
    std::shared_ptr<robot_body_filter::LidarProcessorBase> processor_;
    
    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr removal_marker_pub_;
    
    // TF broadcasting
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    double laser_rotation_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr data_timer_;
    rclcpp::TimerBase::SharedPtr joint_timer_;
    rclcpp::TimerBase::SharedPtr init_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotBodyFilterTutorial>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}