#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PathPlanner : public rlcpp::Node {
  public:
    PatchPlanner() : Node("PathPlanner"){
     goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal",10 , [this](const geometry_msgs::msg::PoseStamped msg){goal = msg;}) 
    }
  private:
    geometry_msgs::msg::PoseStamped goal;
}
