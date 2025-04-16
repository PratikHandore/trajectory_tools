#ifndef TRAJECTORY_SAVER_HPP
#define TRAJECTORY_SAVER_HPP

#include "geometry_msgs/msg/pose_stamped.hpp"
#include <string>
#include <vector>

class TrajectorySaver {
public:
    static bool save_to_json(const std::vector<geometry_msgs::msg::PoseStamped>& trajectory, 
                           const std::string& file_path);
    static bool save_to_csv(const std::vector<geometry_msgs::msg::PoseStamped>& trajectory, 
                          const std::string& file_path);
    static bool save_to_yaml(const std::vector<geometry_msgs::msg::PoseStamped>& trajectory, 
                           const std::string& file_path);
};

#endif  // TRAJECTORY_SAVER_HPP