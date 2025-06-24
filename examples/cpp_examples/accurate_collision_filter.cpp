/**
 * @file accurate_collision_filter.cpp
 * @brief Accurate collision-based point cloud filter
 * 
 * This node creates a more accurate collision filter that only removes points
 * that are actually inside the robot's collision geometry, without being
 * overly conservative with inflation and shadow filtering.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <urdf/model.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shape_operations.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Geometry>

class AccurateCollisionFilter : public rclcpp::Node
{
public:
    AccurateCollisionFilter() : Node("accurate_collision_filter")
    {
        // Declare parameters
        this->declare_parameter("input_topic", "/raw_pointcloud");
        this->declare_parameter("output_topic", "/filtered_pointcloud");
        this->declare_parameter("robot_description", "");
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("collision_padding", 0.02);  // Minimal padding for safety
        this->declare_parameter("max_collision_distance", 2.0);  // Only check nearby points
        this->declare_parameter("check_self_collision_only", true);  // Only remove points inside robot
        
        // Get parameters
        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        robot_description_ = this->get_parameter("robot_description").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        collision_padding_ = this->get_parameter("collision_padding").as_double();
        max_collision_distance_ = this->get_parameter("max_collision_distance").as_double();
        check_self_collision_only_ = this->get_parameter("check_self_collision_only").as_bool();
        
        // Initialize TF
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Load URDF
        if (!loadUrdf()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load URDF model");
            return;
        }
        
        // Setup ROS interfaces (larger queues to prevent message drops)
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 50,
            std::bind(&AccurateCollisionFilter::pointCloudCallback, this, std::placeholders::_1));
        
        filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 50);
        debug_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/collision_debug_markers", 10);
        
        RCLCPP_INFO(this->get_logger(), "AccurateCollisionFilter initialized");
        RCLCPP_INFO(this->get_logger(), "  Input: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Output: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Collision padding: %.3f m", collision_padding_);
        RCLCPP_INFO(this->get_logger(), "  Max collision distance: %.3f m", max_collision_distance_);
        RCLCPP_INFO(this->get_logger(), "  Self-collision only: %s", check_self_collision_only_ ? "true" : "false");
    }

private:
    bool loadUrdf()
    {
        if (robot_description_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No robot_description provided, using simplified collision model");
            createSimplifiedCollisionModel();
            return true;
        }
        
        urdf_model_ = std::make_unique<urdf::Model>();
        if (!urdf_model_->initString(robot_description_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
            return false;
        }
        
        // Extract collision geometries
        extractCollisionGeometries();
        
        RCLCPP_INFO(this->get_logger(), "Loaded URDF with %zu collision bodies", collision_bodies_.size());
        return true;
    }
    
    void createSimplifiedCollisionModel()
    {
        // Create a simple box collision model around base_link
        auto box_shape = std::make_shared<shapes::Box>(1.0, 0.8, 0.6);  // 1m x 0.8m x 0.6m
        auto box_body = std::make_shared<bodies::Box>(box_shape.get());
        box_body->setPadding(collision_padding_);
        
        CollisionBody collision_body;
        collision_body.body = box_body;
        collision_body.link_name = base_frame_;
        collision_body.shape = box_shape;
        
        collision_bodies_.push_back(collision_body);
        
        RCLCPP_INFO(this->get_logger(), "Created simplified collision model: 1.0x0.8x0.6m box");
    }
    
    void extractCollisionGeometries()
    {
        for (const auto& link_pair : urdf_model_->links_) {
            const auto& link = link_pair.second;
            if (!link->collision_array.empty()) {
                for (const auto& collision : link->collision_array) {
                    if (collision->geometry) {
                        auto shape = createShapeFromUrdf(collision->geometry);
                        if (shape) {
                            auto body = createBodyFromShape(shape);
                            if (body) {
                                body->setPadding(collision_padding_);
                                
                                CollisionBody collision_body;
                                collision_body.body = body;
                                collision_body.link_name = link->name;
                                collision_body.shape = shape;
                                collision_body.origin = collision->origin;
                                
                                collision_bodies_.push_back(collision_body);
                            }
                        }
                    }
                }
            }
        }
    }
    
    std::shared_ptr<shapes::Shape> createShapeFromUrdf(const urdf::GeometrySharedPtr& geometry)
    {
        switch (geometry->type) {
            case urdf::Geometry::BOX: {
                auto box = std::dynamic_pointer_cast<urdf::Box>(geometry);
                return std::make_shared<shapes::Box>(box->dim.x, box->dim.y, box->dim.z);
            }
            case urdf::Geometry::CYLINDER: {
                auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(geometry);
                return std::make_shared<shapes::Cylinder>(cylinder->radius, cylinder->length);
            }
            case urdf::Geometry::SPHERE: {
                auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(geometry);
                return std::make_shared<shapes::Sphere>(sphere->radius);
            }
            case urdf::Geometry::MESH: {
                auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(geometry);
                
                // Try to load the actual mesh file
                auto mesh_shape = loadMeshFromFile(mesh->filename, mesh->scale);
                if (mesh_shape) {
                    return mesh_shape;
                } else {
                    // Fallback to bounding box if mesh loading fails
                    RCLCPP_WARN(this->get_logger(), "Failed to load mesh %s, using bounding box approximation", 
                               mesh->filename.c_str());
                    // Use scale to approximate mesh size
                    double size_x = 0.1 * mesh->scale.x;
                    double size_y = 0.1 * mesh->scale.y;
                    double size_z = 0.1 * mesh->scale.z;
                    return std::make_shared<shapes::Box>(size_x, size_y, size_z);
                }
            }
        }
        return nullptr;
    }
    
    std::shared_ptr<bodies::Body> createBodyFromShape(const std::shared_ptr<shapes::Shape>& shape)
    {
        switch (shape->type) {
            case shapes::BOX:
                return std::make_shared<bodies::Box>(shape.get());
            case shapes::CYLINDER:
                return std::make_shared<bodies::Cylinder>(shape.get());
            case shapes::SPHERE:
                return std::make_shared<bodies::Sphere>(shape.get());
            case shapes::MESH:
                return std::make_shared<bodies::ConvexMesh>(shape.get());
            default:
                return nullptr;
        }
    }
    
    std::shared_ptr<shapes::Shape> loadMeshFromFile(const std::string& filename, const urdf::Vector3& scale)
    {
        try {
            // Resolve mesh filename (handle package:// URIs)
            std::string resolved_filename = resolveMeshFilename(filename);
            
            if (resolved_filename.empty()) {
                RCLCPP_WARN(this->get_logger(), "Could not resolve mesh filename: %s", filename.c_str());
                return nullptr;
            }
            
            // Load mesh using geometric_shapes
            shapes::Mesh* mesh = shapes::createMeshFromResource(resolved_filename);
            if (!mesh) {
                RCLCPP_WARN(this->get_logger(), "Failed to load mesh from file: %s", resolved_filename.c_str());
                return nullptr;
            }
            
            // Apply scaling
            if (scale.x != 1.0 || scale.y != 1.0 || scale.z != 1.0) {
                mesh->scale(scale.x, scale.y, scale.z);
            }
            
            RCLCPP_DEBUG(this->get_logger(), "Successfully loaded mesh: %s (vertices: %u, triangles: %u)", 
                        resolved_filename.c_str(), mesh->vertex_count, mesh->triangle_count);
            
            return std::shared_ptr<shapes::Shape>(mesh);
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Exception while loading mesh %s: %s", filename.c_str(), e.what());
            return nullptr;
        }
    }
    
    std::string resolveMeshFilename(const std::string& filename)
    {
        // Handle package:// URIs
        if (filename.find("package://") == 0) {
            // Extract package name and relative path
            size_t package_end = filename.find("/", 10);  // Skip "package://"
            if (package_end == std::string::npos) {
                return "";
            }
            
            std::string package_name = filename.substr(10, package_end - 10);
            std::string relative_path = filename.substr(package_end);
            
            // Try to find package using ament
            try {
                auto package_path = ament_index_cpp::get_package_share_directory(package_name);
                return package_path + relative_path;
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Package not found: %s (error: %s)", package_name.c_str(), e.what());
                return "";
            }
        }
        
        // Handle file:// URIs
        if (filename.find("file://") == 0) {
            return filename.substr(7);  // Remove "file://"
        }
        
        // Handle absolute paths
        if (filename[0] == '/') {
            return filename;
        }
        
        // Handle relative paths (relative to current working directory)
        return filename;
    }
    
    bool transformPointCloudToBaseFrame(const sensor_msgs::msg::PointCloud2& input_msg,
                                       const pcl::PointCloud<pcl::PointXYZ>& input_cloud,
                                       pcl::PointCloud<pcl::PointXYZ>& output_cloud)
    {
        if (input_msg.header.frame_id == base_frame_) {
            // Already in base frame
            output_cloud = input_cloud;
            return true;
        }
        
        try {
            // Get transform from input frame to base frame
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                base_frame_, input_msg.header.frame_id, input_msg.header.stamp, 
                rclcpp::Duration::from_nanoseconds(100000000));
            
            // Convert to Eigen transform
            Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
            transform.translation() = Eigen::Vector3d(
                transform_stamped.transform.translation.x,
                transform_stamped.transform.translation.y,
                transform_stamped.transform.translation.z);
            transform.linear() = Eigen::Quaterniond(
                transform_stamped.transform.rotation.w,
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z).toRotationMatrix();
            
            // Transform point cloud
            output_cloud.clear();
            output_cloud.header = input_cloud.header;
            output_cloud.header.frame_id = base_frame_;
            output_cloud.reserve(input_cloud.size());
            
            for (const auto& point : input_cloud.points) {
                Eigen::Vector3d pt(point.x, point.y, point.z);
                Eigen::Vector3d transformed_pt = transform * pt;
                
                pcl::PointXYZ new_point;
                new_point.x = transformed_pt.x();
                new_point.y = transformed_pt.y();
                new_point.z = transformed_pt.z();
                output_cloud.push_back(new_point);
            }
            
            output_cloud.width = output_cloud.size();
            output_cloud.height = 1;
            output_cloud.is_dense = input_cloud.is_dense;
            
            return true;
            
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), 
                "Transform from %s to %s failed: %s", 
                input_msg.header.frame_id.c_str(), base_frame_.c_str(), ex.what());
            return false;
        }
    }
    
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert to PCL
        pcl::PointCloud<pcl::PointXYZ> input_cloud;
        pcl::fromROSMsg(*msg, input_cloud);
        
        // Transform point cloud to base_frame if needed
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        if (!transformPointCloudToBaseFrame(*msg, input_cloud, transformed_cloud)) {
            RCLCPP_WARN(this->get_logger(), "Failed to transform point cloud to base frame, passing through all points");
            filtered_pub_->publish(*msg);
            return;
        }
        
        // Create output cloud
        pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
        filtered_cloud.header = transformed_cloud.header;
        filtered_cloud.reserve(transformed_cloud.size());
        
        // Get transforms for all collision bodies (now all relative to base_frame)
        std::vector<Eigen::Isometry3d> body_transforms;
        if (!getCollisionTransforms(msg->header.stamp, body_transforms)) {
            RCLCPP_WARN(this->get_logger(), "Failed to get collision transforms, passing through all points");
            filtered_pub_->publish(*msg);
            return;
        }
        
        // Filter points
        size_t removed_count = 0;
        for (const auto& point : transformed_cloud.points) {
            if (!isPointInCollision(point, body_transforms)) {
                filtered_cloud.push_back(point);
            } else {
                removed_count++;
            }
        }
        
        // Publish filtered cloud
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(filtered_cloud, output_msg);
        output_msg.header = msg->header;
        filtered_pub_->publish(output_msg);
        
        // Log statistics
        static int log_counter = 0;
        if (++log_counter % 20 == 0) {
            double removal_rate = (double)removed_count / transformed_cloud.size() * 100.0;
            RCLCPP_INFO(this->get_logger(), 
                "Filtered: %zu -> %zu points (%.1f%% removed)",
                transformed_cloud.size(), filtered_cloud.size(), removal_rate);
        }
        
        // Publish debug markers
        publishDebugMarkers(msg->header);
    }
    
    bool getCollisionTransforms(const builtin_interfaces::msg::Time& stamp, 
                               std::vector<Eigen::Isometry3d>& transforms)
    {
        transforms.clear();
        transforms.reserve(collision_bodies_.size());
        
        for (const auto& body : collision_bodies_) {
            try {
                geometry_msgs::msg::TransformStamped transform_stamped;
                
                // Get transform from sensor frame to link frame
                if (body.link_name == base_frame_) {
                    // Use identity for base frame
                    transform_stamped.transform.rotation.w = 1.0;
                } else {
                    transform_stamped = tf_buffer_->lookupTransform(
                        base_frame_, body.link_name, stamp, rclcpp::Duration::from_nanoseconds(100000000));
                }
                
                Eigen::Isometry3d eigen_transform = Eigen::Isometry3d::Identity();
                eigen_transform.translation() = Eigen::Vector3d(
                    transform_stamped.transform.translation.x,
                    transform_stamped.transform.translation.y,
                    transform_stamped.transform.translation.z);
                eigen_transform.linear() = Eigen::Quaterniond(
                    transform_stamped.transform.rotation.w,
                    transform_stamped.transform.rotation.x,
                    transform_stamped.transform.rotation.y,
                    transform_stamped.transform.rotation.z).toRotationMatrix();
                
                // Apply URDF collision origin offset
                if (body.origin.position.x != 0.0 || body.origin.position.y != 0.0 || body.origin.position.z != 0.0 ||
                    body.origin.rotation.x != 0.0 || body.origin.rotation.y != 0.0 || body.origin.rotation.z != 0.0 || body.origin.rotation.w != 1.0) {
                    
                    Eigen::Isometry3d origin_transform = Eigen::Isometry3d::Identity();
                    origin_transform.translation() = Eigen::Vector3d(
                        body.origin.position.x, body.origin.position.y, body.origin.position.z);
                    origin_transform.linear() = Eigen::Quaterniond(
                        body.origin.rotation.w, body.origin.rotation.x, body.origin.rotation.y, body.origin.rotation.z).toRotationMatrix();
                    
                    eigen_transform = eigen_transform * origin_transform;
                }
                
                transforms.push_back(eigen_transform);
                
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), "Transform failed for link %s: %s", body.link_name.c_str(), ex.what());
                return false;
            }
        }
        
        return true;
    }
    
    bool isPointInCollision(const pcl::PointXYZ& point, 
                           const std::vector<Eigen::Isometry3d>& body_transforms)
    {
        Eigen::Vector3d eigen_point(point.x, point.y, point.z);
        
        // Quick distance check - only check points near the robot
        if (eigen_point.norm() > max_collision_distance_) {
            return false;
        }
        
        for (size_t i = 0; i < collision_bodies_.size(); ++i) {
            const auto& body = collision_bodies_[i];
            const auto& transform = body_transforms[i];
            
            // Transform point to body frame
            Eigen::Vector3d transformed_point = transform.inverse() * eigen_point;
            
            // Check if point is inside collision body
            if (body.body->containsPoint(transformed_point)) {
                return true;
            }
        }
        
        return false;
    }
    
    void publishDebugMarkers(const std_msgs::msg::Header& header)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        for (size_t i = 0; i < collision_bodies_.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.header.frame_id = base_frame_;
            marker.ns = "collision_bodies";
            marker.id = static_cast<int>(i);
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.color.a = 0.3;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            
            const auto& body = collision_bodies_[i];
            
            switch (body.shape->type) {
                case shapes::BOX: {
                    auto box = std::static_pointer_cast<shapes::Box>(body.shape);
                    marker.type = visualization_msgs::msg::Marker::CUBE;
                    marker.scale.x = box->size[0] + 2 * collision_padding_;
                    marker.scale.y = box->size[1] + 2 * collision_padding_;
                    marker.scale.z = box->size[2] + 2 * collision_padding_;
                    break;
                }
                case shapes::CYLINDER: {
                    auto cylinder = std::static_pointer_cast<shapes::Cylinder>(body.shape);
                    marker.type = visualization_msgs::msg::Marker::CYLINDER;
                    marker.scale.x = 2 * (cylinder->radius + collision_padding_);
                    marker.scale.y = 2 * (cylinder->radius + collision_padding_);
                    marker.scale.z = cylinder->length + 2 * collision_padding_;
                    break;
                }
                case shapes::SPHERE: {
                    auto sphere = std::static_pointer_cast<shapes::Sphere>(body.shape);
                    marker.type = visualization_msgs::msg::Marker::SPHERE;
                    marker.scale.x = 2 * (sphere->radius + collision_padding_);
                    marker.scale.y = 2 * (sphere->radius + collision_padding_);
                    marker.scale.z = 2 * (sphere->radius + collision_padding_);
                    break;
                }
                case shapes::MESH: {
                    auto mesh = std::static_pointer_cast<shapes::Mesh>(body.shape);
                    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
                    
                    // Set mesh data
                    marker.points.reserve(mesh->triangle_count * 3);
                    for (unsigned int i = 0; i < mesh->triangle_count; ++i) {
                        for (int j = 0; j < 3; ++j) {
                            unsigned int vertex_idx = mesh->triangles[i * 3 + j];
                            geometry_msgs::msg::Point point;
                            point.x = mesh->vertices[vertex_idx * 3];
                            point.y = mesh->vertices[vertex_idx * 3 + 1];
                            point.z = mesh->vertices[vertex_idx * 3 + 2];
                            marker.points.push_back(point);
                        }
                    }
                    
                    // For triangle list, scale is not used in the same way
                    marker.scale.x = 1.0;
                    marker.scale.y = 1.0;
                    marker.scale.z = 1.0;
                    break;
                }
            }
            
            // Set pose from URDF origin
            marker.pose.position.x = body.origin.position.x;
            marker.pose.position.y = body.origin.position.y;
            marker.pose.position.z = body.origin.position.z;
            marker.pose.orientation.x = body.origin.rotation.x;
            marker.pose.orientation.y = body.origin.rotation.y;
            marker.pose.orientation.z = body.origin.rotation.z;
            marker.pose.orientation.w = body.origin.rotation.w;
            
            marker_array.markers.push_back(marker);
        }
        
        debug_pub_->publish(marker_array);
    }
    
    struct CollisionBody {
        std::shared_ptr<bodies::Body> body;
        std::string link_name;
        std::shared_ptr<shapes::Shape> shape;
        urdf::Pose origin;
    };
    
    // Parameters
    std::string input_topic_;
    std::string output_topic_;
    std::string robot_description_;
    std::string base_frame_;
    double collision_padding_;
    double max_collision_distance_;
    bool check_self_collision_only_;
    
    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
    
    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Robot model
    std::unique_ptr<urdf::Model> urdf_model_;
    std::vector<CollisionBody> collision_bodies_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AccurateCollisionFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}