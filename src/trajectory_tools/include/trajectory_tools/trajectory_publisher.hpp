#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "trajectory_tools/srv/save_trajectory.hpp"
#include <deque>
#include <memory>

class TrajectoryPublisher : public rclcpp::Node {
public:
    TrajectoryPublisher();

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void save_trajectory_callback(
        const std::shared_ptr<trajectory_tools::srv::SaveTrajectory::Request> request,
        std::shared_ptr<trajectory_tools::srv::SaveTrajectory::Response> response);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Service<trajectory_tools::srv::SaveTrajectory>::SharedPtr save_service_;
    
    std::deque<geometry_msgs::msg::PoseStamped> trajectory_;
    rclcpp::Time last_save_time_;
};