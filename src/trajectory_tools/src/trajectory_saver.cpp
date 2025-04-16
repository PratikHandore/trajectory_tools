#include "trajectory_tools/trajectory_saver.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

bool TrajectorySaver::save_to_json(const std::vector<geometry_msgs::msg::PoseStamped>& trajectory, 
                                 const std::string& file_path) {
  try {
    nlohmann::json j;
    for (const auto& pose : trajectory) {
      nlohmann::json p;
      p["header"]["frame_id"] = pose.header.frame_id;
      p["pose"]["position"]["x"] = pose.pose.position.x;
      p["pose"]["position"]["y"] = pose.pose.position.y;
      p["pose"]["position"]["z"] = pose.pose.position.z;
      p["pose"]["orientation"]["x"] = pose.pose.orientation.x;
      p["pose"]["orientation"]["y"] = pose.pose.orientation.y;
      p["pose"]["orientation"]["z"] = pose.pose.orientation.z;
      p["pose"]["orientation"]["w"] = pose.pose.orientation.w;
      j["trajectory"].push_back(p);
    }
    
    std::ofstream file(file_path);
    file << j.dump(4);
    return file.good();
  } catch (...) {
    return false;
  }
}

bool TrajectorySaver::save_to_csv(const std::vector<geometry_msgs::msg::PoseStamped>& trajectory,
                                const std::string& file_path) {
  try {
    std::ofstream file(file_path);
    file << "frame_id,x,y,z,qx,qy,qz,qw\n";
    for (const auto& pose : trajectory) {
      file << pose.header.frame_id << ","
           << pose.pose.position.x << ","
           << pose.pose.position.y << ","
           << pose.pose.position.z << ","
           << pose.pose.orientation.x << ","
           << pose.pose.orientation.y << ","
           << pose.pose.orientation.z << ","
           << pose.pose.orientation.w << "\n";
    }
    return file.good();
  } catch (...) {
    return false;
  }
}

bool TrajectorySaver::save_to_yaml(const std::vector<geometry_msgs::msg::PoseStamped>& trajectory,
                                 const std::string& file_path) {
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "trajectory" << YAML::Value << YAML::BeginSeq;
    
    for (const auto& pose : trajectory) {
      out << YAML::BeginMap;
      out << YAML::Key << "header" << YAML::Value 
          << YAML::BeginMap << YAML::Key << "frame_id" << YAML::Value << pose.header.frame_id << YAML::EndMap;
      out << YAML::Key << "pose" << YAML::Value 
          << YAML::BeginMap
          << YAML::Key << "position" << YAML::Value 
            << YAML::BeginMap
            << YAML::Key << "x" << YAML::Value << pose.pose.position.x
            << YAML::Key << "y" << YAML::Value << pose.pose.position.y
            << YAML::Key << "z" << YAML::Value << pose.pose.position.z
            << YAML::EndMap
          << YAML::Key << "orientation" << YAML::Value 
            << YAML::BeginMap
            << YAML::Key << "x" << YAML::Value << pose.pose.orientation.x
            << YAML::Key << "y" << YAML::Value << pose.pose.orientation.y
            << YAML::Key << "z" << YAML::Value << pose.pose.orientation.z
            << YAML::Key << "w" << YAML::Value << pose.pose.orientation.w
            << YAML::EndMap
          << YAML::EndMap;
      out << YAML::EndMap;
    }
    
    out << YAML::EndSeq;
    out << YAML::EndMap;
    
    std::ofstream file(file_path);
    file << out.c_str();
    return file.good();
  } catch (...) {
    return false;
  }
}