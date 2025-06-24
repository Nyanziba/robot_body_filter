/**
 * @file simple_pointcloud_publisher.cpp
 * @brief Simple point cloud publisher for testing collision filters
 * 
 * This node publishes synthetic point cloud data for testing robot body filters.
 * It generates random points in a specified range and publishes them at a configurable rate.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>
#include <cmath>

class SimplePointCloudPublisher : public rclcpp::Node
{
public:
    SimplePointCloudPublisher() : Node("simple_pointcloud_publisher")
    {
        // Declare parameters
        this->declare_parameter("output_topic", "/raw_pointcloud");
        this->declare_parameter("frame_id", "laser");
        this->declare_parameter("publish_rate", 10.0);
        this->declare_parameter("num_points", 50000);
        this->declare_parameter("point_range", 3.0);
        this->declare_parameter("add_noise", true);
        this->declare_parameter("noise_stddev", 0.01);
        this->declare_parameter("pattern_type", "grid");  // "grid", "sphere", "cylinder", "random"
        this->declare_parameter("grid_resolution", 0.1);
        
        // Get parameters
        output_topic_ = this->get_parameter("output_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        num_points_ = this->get_parameter("num_points").as_int();
        point_range_ = this->get_parameter("point_range").as_double();
        add_noise_ = this->get_parameter("add_noise").as_bool();
        noise_stddev_ = this->get_parameter("noise_stddev").as_double();
        pattern_type_ = this->get_parameter("pattern_type").as_string();
        grid_resolution_ = this->get_parameter("grid_resolution").as_double();
        
        // Setup publisher
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
        
        // Setup timer
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&SimplePointCloudPublisher::publishPointCloud, this));
        
        // Initialize random number generator
        rng_.seed(std::random_device{}());
        
        RCLCPP_INFO(this->get_logger(), "SimplePointCloudPublisher initialized");
        RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Publish rate: %.1f Hz", publish_rate_);
        RCLCPP_INFO(this->get_logger(), "  Points per cloud: %d", num_points_);
        RCLCPP_INFO(this->get_logger(), "  Point range: ±%.1f m", point_range_);
        RCLCPP_INFO(this->get_logger(), "  Pattern type: %s", pattern_type_.c_str());
        if (pattern_type_ == "grid") {
            RCLCPP_INFO(this->get_logger(), "  Grid resolution: %.3f m", grid_resolution_);
        }
        RCLCPP_INFO(this->get_logger(), "  Add noise: %s", add_noise_ ? "true" : "false");
        if (add_noise_) {
            RCLCPP_INFO(this->get_logger(), "  Noise std dev: %.3f m", noise_stddev_);
        }
    }

private:
    void publishPointCloud()
    {
        // Create PCL point cloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.header.frame_id = frame_id_;
        cloud.header.stamp = pcl_conversions::toPCL(this->now());
        
        // Generate points based on pattern type
        if (pattern_type_ == "grid") {
            generateGridPoints(cloud);
        } else if (pattern_type_ == "sphere") {
            generateSpherePoints(cloud);
        } else if (pattern_type_ == "cylinder") {
            generateCylinderPoints(cloud);
        } else {
            generateRandomPoints(cloud);
        }
        
        // Convert to ROS message
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = frame_id_;
        
        // Publish
        pointcloud_pub_->publish(cloud_msg);
        
        // Log statistics
        static int publish_count = 0;
        if (++publish_count % static_cast<int>(publish_rate_ * 5) == 0) {  // Every 5 seconds
            RCLCPP_INFO(this->get_logger(), 
                "Published %d point clouds with %d points each", 
                publish_count, num_points_);
        }
    }
    
    void generateRandomPoints(pcl::PointCloud<pcl::PointXYZ>& cloud)
    {
        cloud.points.clear();
        cloud.points.reserve(num_points_);
        
        // Random distributions
        std::uniform_real_distribution<double> coord_dist(-point_range_, point_range_);
        std::normal_distribution<double> noise_dist(0.0, noise_stddev_);
        
        for (int i = 0; i < num_points_; ++i) {
            pcl::PointXYZ point;
            
            // Generate random coordinates
            point.x = coord_dist(rng_);
            point.y = coord_dist(rng_);
            point.z = coord_dist(rng_);
            
            // Add noise if enabled
            if (add_noise_) {
                point.x += noise_dist(rng_);
                point.y += noise_dist(rng_);
                point.z += noise_dist(rng_);
            }
            
            cloud.points.push_back(point);
        }
        
        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = !add_noise_;  // Dense if no noise, non-dense if noise might create invalid points
    }
    
    void generateGridPoints(pcl::PointCloud<pcl::PointXYZ>& cloud)
    {
        cloud.points.clear();
        
        // Calculate number of points per dimension
        int points_per_dim = std::ceil(std::cbrt(num_points_));
        double step = grid_resolution_;
        double half_range = (points_per_dim - 1) * step * 0.5;
        
        // Ensure we stay within point_range_
        if (half_range > point_range_) {
            step = 2.0 * point_range_ / (points_per_dim - 1);
            half_range = point_range_;
        }
        
        std::normal_distribution<double> noise_dist(0.0, noise_stddev_);
        
        for (int x_idx = 0; x_idx < points_per_dim && cloud.points.size() < static_cast<size_t>(num_points_); ++x_idx) {
            for (int y_idx = 0; y_idx < points_per_dim && cloud.points.size() < static_cast<size_t>(num_points_); ++y_idx) {
                for (int z_idx = 0; z_idx < points_per_dim && cloud.points.size() < static_cast<size_t>(num_points_); ++z_idx) {
                    pcl::PointXYZ point;
                    
                    // Calculate grid position
                    point.x = -half_range + x_idx * step;
                    point.y = -half_range + y_idx * step;
                    point.z = -half_range + z_idx * step;
                    
                    // Add noise if enabled
                    if (add_noise_) {
                        point.x += noise_dist(rng_);
                        point.y += noise_dist(rng_);
                        point.z += noise_dist(rng_);
                    }
                    
                    cloud.points.push_back(point);
                }
            }
        }
        
        RCLCPP_DEBUG_ONCE(this->get_logger(), "Grid: %d×%d×%d, step=%.3fm, generated %zu points", 
                          points_per_dim, points_per_dim, points_per_dim, step, cloud.points.size());
    }
    
    void generateSpherePoints(pcl::PointCloud<pcl::PointXYZ>& cloud)
    {
        cloud.points.clear();
        cloud.points.reserve(num_points_);
        
        std::normal_distribution<double> noise_dist(0.0, noise_stddev_);
        
        // Generate points on sphere surfaces at different radii
        int num_spheres = 10;
        int points_per_sphere = num_points_ / num_spheres;
        
        for (int sphere_idx = 0; sphere_idx < num_spheres; ++sphere_idx) {
            double radius = point_range_ * (sphere_idx + 1) / num_spheres;
            
            for (int i = 0; i < points_per_sphere && cloud.points.size() < static_cast<size_t>(num_points_); ++i) {
                // Use golden spiral method for even distribution on sphere
                double theta = 2.0 * M_PI * i / 1.618033988749895;  // Golden ratio
                double phi = std::acos(1.0 - 2.0 * i / points_per_sphere);
                
                pcl::PointXYZ point;
                point.x = radius * std::sin(phi) * std::cos(theta);
                point.y = radius * std::sin(phi) * std::sin(theta);
                point.z = radius * std::cos(phi);
                
                // Add noise if enabled
                if (add_noise_) {
                    point.x += noise_dist(rng_);
                    point.y += noise_dist(rng_);
                    point.z += noise_dist(rng_);
                }
                
                cloud.points.push_back(point);
            }
        }
        
        RCLCPP_DEBUG_ONCE(this->get_logger(), "Sphere: %d spheres, generated %zu points", 
                          num_spheres, cloud.points.size());
    }
    
    void generateCylinderPoints(pcl::PointCloud<pcl::PointXYZ>& cloud)
    {
        cloud.points.clear();
        cloud.points.reserve(num_points_);
        
        std::normal_distribution<double> noise_dist(0.0, noise_stddev_);
        
        // Generate points on cylindrical surfaces
        int num_cylinders = 20;
        int points_per_cylinder = num_points_ / num_cylinders;
        
        for (int cyl_idx = 0; cyl_idx < num_cylinders; ++cyl_idx) {
            double radius = point_range_ * (cyl_idx + 1) / num_cylinders;
            
            for (int i = 0; i < points_per_cylinder && cloud.points.size() < static_cast<size_t>(num_points_); ++i) {
                double theta = 2.0 * M_PI * i / points_per_cylinder;
                double z = -point_range_ + 2.0 * point_range_ * i / points_per_cylinder;
                
                pcl::PointXYZ point;
                point.x = radius * std::cos(theta);
                point.y = radius * std::sin(theta);
                point.z = z;
                
                // Add noise if enabled
                if (add_noise_) {
                    point.x += noise_dist(rng_);
                    point.y += noise_dist(rng_);
                    point.z += noise_dist(rng_);
                }
                
                cloud.points.push_back(point);
            }
        }
        
        RCLCPP_DEBUG_ONCE(this->get_logger(), "Cylinder: %d cylinders, generated %zu points", 
                          num_cylinders, cloud.points.size());
    }
    
    // Parameters
    std::string output_topic_;
    std::string frame_id_;
    double publish_rate_;
    int num_points_;
    double point_range_;
    bool add_noise_;
    double noise_stddev_;
    std::string pattern_type_;
    double grid_resolution_;
    
    // ROS interfaces
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Random number generation
    std::mt19937 rng_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplePointCloudPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}