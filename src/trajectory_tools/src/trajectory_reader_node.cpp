#include "trajectory_tools/trajectory_reader.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <vector>
#include <string>

TrajectoryReader::TrajectoryReader() : Node("trajectory_reader") {
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/loaded_trajectory_markers", 10);
    load_service_ = create_service<trajectory_tools::srv::LoadTrajectory>(
        "load_trajectory", 
        std::bind(&TrajectoryReader::load_trajectory_callback, this, 
                 std::placeholders::_1, std::placeholders::_2));
}

void TrajectoryReader::load_trajectory_callback(
    const std::shared_ptr<trajectory_tools::srv::LoadTrajectory::Request> request,
    std::shared_ptr<trajectory_tools::srv::LoadTrajectory::Response> response) {
    
    std::vector<geometry_msgs::msg::PoseStamped> trajectory;
    bool success = false;
    
    try {
        if (request->format == "json") {
            std::ifstream file(request->file_path);
            nlohmann::json json_data;
            file >> json_data;
            
            for (const auto& pose_json : json_data["trajectory"]) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = pose_json["header"]["frame_id"];
                pose.pose.position.x = pose_json["pose"]["position"]["x"];
                pose.pose.position.y = pose_json["pose"]["position"]["y"];
                pose.pose.position.z = pose_json["pose"]["position"]["z"];
                pose.pose.orientation.x = pose_json["pose"]["orientation"]["x"];
                pose.pose.orientation.y = pose_json["pose"]["orientation"]["y"];
                pose.pose.orientation.z = pose_json["pose"]["orientation"]["z"];
                pose.pose.orientation.w = pose_json["pose"]["orientation"]["w"];
                trajectory.push_back(pose);
            }
            success = true;
        } else if (request->format == "csv") {
            std::ifstream file(request->file_path);
            std::string line;
            // Skip header
            std::getline(file, line);
            
            while (std::getline(file, line)) {
                std::stringstream ss(line);
                std::string item;
                geometry_msgs::msg::PoseStamped pose;
                
                std::getline(ss, item, ',');
                pose.header.frame_id = item;
                
                std::getline(ss, item, ',');
                pose.pose.position.x = std::stod(item);
                std::getline(ss, item, ',');
                pose.pose.position.y = std::stod(item);
                std::getline(ss, item, ',');
                pose.pose.position.z = std::stod(item);
                
                std::getline(ss, item, ',');
                pose.pose.orientation.x = std::stod(item);
                std::getline(ss, item, ',');
                pose.pose.orientation.y = std::stod(item);
                std::getline(ss, item, ',');
                pose.pose.orientation.z = std::stod(item);
                std::getline(ss, item);
                pose.pose.orientation.w = std::stod(item);
                
                trajectory.push_back(pose);
            }
            success = true;
        } else if (request->format == "yaml") {
            YAML::Node yaml_data = YAML::LoadFile(request->file_path);
            
            for (const auto& pose_yaml : yaml_data["trajectory"]) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = pose_yaml["header"]["frame_id"].as<std::string>();
                pose.pose.position.x = pose_yaml["pose"]["position"]["x"].as<double>();
                pose.pose.position.y = pose_yaml["pose"]["position"]["y"].as<double>();
                pose.pose.position.z = pose_yaml["pose"]["position"]["z"].as<double>();
                pose.pose.orientation.x = pose_yaml["pose"]["orientation"]["x"].as<double>();
                pose.pose.orientation.y = pose_yaml["pose"]["orientation"]["y"].as<double>();
                pose.pose.orientation.z = pose_yaml["pose"]["orientation"]["z"].as<double>();
                pose.pose.orientation.w = pose_yaml["pose"]["orientation"]["w"].as<double>();
                trajectory.push_back(pose);
            }
            success = true;
        }
        
        if (success) {
            publish_trajectory(trajectory);
            response->message = "Trajectory loaded and published successfully";
        } else {
            response->message = "Unsupported format or failed to parse file";
        }
    } catch (const std::exception& e) {
        response->message = std::string("Error: ") + e.what();
    }
    
    response->success = success;
}

void TrajectoryReader::publish_trajectory(const std::vector<geometry_msgs::msg::PoseStamped>& trajectory) {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = now();
    marker.ns = "loaded_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    
    for (const auto& pose : trajectory) {
        marker.points.push_back(pose.pose.position);
    }
    
    marker_array.markers.push_back(marker);
    marker_pub_->publish(marker_array);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryReader>());
    rclcpp::shutdown();
    return 0;
}