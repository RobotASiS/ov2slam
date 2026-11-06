#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/empty.hpp"
#include <cmath>

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
                goal_active = true;
            }
        );

        // Odometria - zeby wiedziec kiedy przestac przeliczac trase (cel osiagniety)
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                robot_x = msg->pose.pose.position.x;
                robot_y = msg->pose.pose.position.y;
                has_odom = true;
            }
        );

        // MPC daje znac, ze cel osiagniety albo sie poddal - koniec przeliczania
        cancel_sub = this->create_subscription<std_msgs::msg::Empty>(
            "/goal_cancel", 10,
            [this](const std_msgs::msg::Empty::SharedPtr) {
                if (goal_active) {
                    RCLCPP_INFO(this->get_logger(), "Cel anulowany - koniec przeliczania trasy");
                }
                goal_active = false;
                has_goal = false;
            }
        );

        this->declare_parameter("replan_period", 1.0);
        this->declare_parameter("goal_tolerance", 0.15);
        replan_period = this->get_parameter("replan_period").as_double();
        goal_tolerance = this->get_parameter("goal_tolerance").as_double();

        // Cykliczne przeliczanie trasy, zeby uwzglednic nowe przeszkody na mapie
        replan_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(replan_period),
            std::bind(&PathPlanner::replan, this)
        );

        action_client = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
            this, "compute_path_to_pose"
        );

        path_pub = this->create_publisher<nav_msgs::msg::Path>("/plan", 10);

        this->timer_ = this->create_wall_timer(
            200ms, std::bind(&PathPlanner::create_path, this)
        );
    }

private:
    void replan() {
        if (!goal_active || has_goal || goal_in_progress_) {
            return;
        }
        if (has_odom && std::hypot(goal.pose.position.x - robot_x, goal.pose.position.y - robot_y) <= goal_tolerance) {
            RCLCPP_INFO(this->get_logger(), "Cel osiagniety - koniec przeliczania trasy");
            goal_active = false;
            return;
        }
        replanning_ = true;
        has_goal = true;   // create_path() wysle zapytanie przy najblizszym ticku
    }

    void create_path() {
        if (!has_goal || goal_in_progress_) {
            return;
        }

        if (!this->action_client->wait_for_action_server(200ms)) {
            RCLCPP_WARN(this->get_logger(), "czeka na serwer");
            return;
        }

        has_goal = false;
        ComputePathToPose::Goal goal_path;
        goal_path.goal = this->goal;
        goal_path.goal.header.stamp = this->now();

        if (!replanning_) {
            RCLCPP_INFO(this->get_logger(), "Wysyłam zapytanie");
        }
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
            if (!replanning_) {
                RCLCPP_INFO(this->get_logger(), "Cel zaakceptowany, planner oblicza trasę...");
            }
        }
    }

    void result_callback(const GoalHandleComputePath::WrappedResult & result) {
        this->goal_in_progress_ = false;
        const bool was_replan = replanning_;
        replanning_ = false;

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                if (!was_replan) {
                    RCLCPP_INFO(this->get_logger(), "Trasa znaleziona sukcesem!");
                }
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
        if (!goal_active) {
            return;   // cel anulowany w trakcie liczenia - nie wysylaj starej trasy
        }
        if (was_replan) {
            path_pub->publish(path);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Ścieżka zawiera %zu punktów pośrednich.", path.poses.size());

        if (!path.poses.empty()) {
            auto start_point = path.poses.front().pose.position;
            auto end_point = path.poses.back().pose.position;
            RCLCPP_INFO(this->get_logger(), "Start: [%.2f, %.2f] -> Koniec: [%.2f, %.2f]",
                        start_point.x, start_point.y, end_point.x, end_point.y);
        }

        path_pub->publish(path);
    }

    geometry_msgs::msg::PoseStamped goal;
    bool has_goal = false;
    bool goal_in_progress_ = false;
    bool goal_active = false;      // cel ustawiony i jeszcze nie osiagniety
    bool replanning_ = false;
    bool has_odom = false;
    double robot_x = 0.0, robot_y = 0.0;
    double replan_period = 1.0;
    double goal_tolerance = 0.15;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub;
    rclcpp::TimerBase::SharedPtr replan_timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
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


