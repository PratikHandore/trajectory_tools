#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "trajectory_tools/srv/load_trajectory.hpp"

#include <vector>

class TrajectoryReader : public rclcpp::Node {
public:
    TrajectoryReader();

private:
    void load_trajectory_callback(
        const std::shared_ptr<trajectory_tools::srv::LoadTrajectory::Request> request,
        std::shared_ptr<trajectory_tools::srv::LoadTrajectory::Response> response);
    void publish_trajectory(const std::vector<geometry_msgs::msg::PoseStamped>& trajectory);

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Service<trajectory_tools::srv::LoadTrajectory>::SharedPtr load_service_;
};