#include "trajectory_tools/trajectory_publisher.hpp"
#include "trajectory_tools/trajectory_saver.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

TrajectoryPublisher::TrajectoryPublisher() : Node("trajectory_publisher") {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&TrajectoryPublisher::odom_callback, this, std::placeholders::_1));
    
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/trajectory_markers", 10);
    save_service_ = create_service<trajectory_tools::srv::SaveTrajectory>(
        "save_trajectory", 
        std::bind(&TrajectoryPublisher::save_trajectory_callback, this, 
                 std::placeholders::_1, std::placeholders::_2));
}

void TrajectoryPublisher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    trajectory_.push_back(pose);
    
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header = msg->header;
    marker.ns = "trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    
    for (const auto& pose : trajectory_) {
        marker.points.push_back(pose.pose.position);
    }
    
    marker_array.markers.push_back(marker);
    marker_pub_->publish(marker_array);
}

void TrajectoryPublisher::save_trajectory_callback(
    const std::shared_ptr<trajectory_tools::srv::SaveTrajectory::Request> request,
    std::shared_ptr<trajectory_tools::srv::SaveTrajectory::Response> response) {
    
    std::vector<geometry_msgs::msg::PoseStamped> trajectory_to_save;
    
    if (request->duration > 0) {
        rclcpp::Time cutoff_time = now() - rclcpp::Duration::from_seconds(request->duration);
        for (const auto& pose : trajectory_) {
            if (rclcpp::Time(pose.header.stamp) >= cutoff_time) {
                trajectory_to_save.push_back(pose);
            }
        }
    } else {
        trajectory_to_save.assign(trajectory_.begin(), trajectory_.end());
    }
    
    bool success = false;
    if (request->format == "json") {
        success = TrajectorySaver::save_to_json(trajectory_to_save, request->file_path);
    } else if (request->format == "csv") {
        success = TrajectorySaver::save_to_csv(trajectory_to_save, request->file_path);
    } else if (request->format == "yaml") {
        success = TrajectorySaver::save_to_yaml(trajectory_to_save, request->file_path);
    }
    
    response->success = success;
    response->message = success ? "Trajectory saved successfully" : "Failed to save trajectory";
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}