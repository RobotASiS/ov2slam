#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class PathPlanner : public rclcpp::Node {
public:
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

    PathPlanner() : Node("PathPlanner") {
        goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                goal = *msg;
                has_goal = true;
            }
        );

        action_client = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
            this, "compute_path_to_pose"
        );

        this->timer_ = this->create_wall_timer(
            200ms, std::bind(&PathPlanner::create_path, this)
        );
    }

private:
    void create_path() {
        if (!has_goal || goal_in_progress_) {
            return;
        }

        if (!this->action_client->wait_for_action_server(200ms)) {
            RCLCPP_WARN(this->get_logger(), "czeka na serwer");
            return;
        }

        ComputePathToPose::Goal goal_path;
        goal_path.goal = this->goal;
        goal_path.goal.header.stamp = this->now();

        RCLCPP_INFO(this->get_logger(), "Wysyłam zapytanie");
        this->goal_in_progress_ = true;

        rclcpp_action::Client<ComputePathToPose>::SendGoalOptions send_goal_options;

        send_goal_options.goal_response_callback =
            std::bind(&PathPlanner::goal_response_callback, this, std::placeholders::_1);

        send_goal_options.result_callback =
            std::bind(&PathPlanner::result_callback, this, std::placeholders::_1);

        this->action_client->async_send_goal(goal_path, send_goal_options);
    }

    void goal_response_callback(const GoalHandleComputePath::SharedPtr & goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Cel odrzucony przez planner (np. cel poza mapą)!");
            this->goal_in_progress_ = false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Cel zaakceptowany, planner oblicza trasę...");
        }
    }

    void result_callback(const GoalHandleComputePath::WrappedResult & result) {
        this->goal_in_progress_ = false;

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Trasa znaleziona sukcesem!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Nie udało się znaleźć ścieżki (brak wolnej drogi)!");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Zapytanie anulowane.");
                return;
            default:
                return;
        }

        const auto & path = result.result->path;
        RCLCPP_INFO(this->get_logger(), "Ścieżka zawiera %zu punktów pośrednich.", path.poses.size());

        if (!path.poses.empty()) {
            auto start_point = path.poses.front().pose.position;
            auto end_point = path.poses.back().pose.position;
            RCLCPP_INFO(this->get_logger(), "Start: [%.2f, %.2f] -> Koniec: [%.2f, %.2f]",
                        start_point.x, start_point.y, end_point.x, end_point.y);
        }
    }

    geometry_msgs::msg::PoseStamped goal;
    bool has_goal = false;
    bool goal_in_progress_ = false;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr action_client;    
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


