#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <casadi/casadi.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;
using namespace casadi;

class CasadiMpcNode : public rclcpp::Node {
public:
    CasadiMpcNode() : Node("casadi_mpc_node") {
        this->declare_parameter<double>("wheelbase", 0.20);
        this->declare_parameter<double>("max_speed", 0.40);
        this->declare_parameter<double>("min_speed", 0.0);
        this->declare_parameter<double>("max_steer", 0.60);
        this->declare_parameter<int>("horizon_n", 10);
        this->declare_parameter<double>("dt", 0.1);
        this->declare_parameter<double>("goal_tolerance", 0.15);
        this->declare_parameter<double>("goal_timeout_sec", 15.0); // Failsafe: zatrzymanie po 15 sek bez osiągnięcia celu

        L_ = this->get_parameter("wheelbase").as_double();
        v_max_ = this->get_parameter("max_speed").as_double();
        v_min_ = this->get_parameter("min_speed").as_double();
        delta_max_ = this->get_parameter("max_steer").as_double();
        N_ = this->get_parameter("horizon_n").as_int();
        dt_ = this->get_parameter("dt").as_double();
        goal_tol_ = this->get_parameter("goal_tolerance").as_double();
        goal_timeout_sec_ = this->get_parameter("goal_timeout_sec").as_double();

        setup_mpc();

        plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&CasadiMpcNode::plan_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&CasadiMpcNode::odom_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pred_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/mpc/predicted_path", 10);

        control_timer_ = this->create_wall_timer(
            100ms, std::bind(&CasadiMpcNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "C++ CasADi MPC Node zainicjalizowany (N=%d, dt=%.2f, L=%.2f, Failsafe=%.1fs)", 
                    N_, dt_, L_, goal_timeout_sec_);
    }

private:
    double L_, v_max_, v_min_, delta_max_, dt_, goal_tol_, goal_timeout_sec_;
    int N_;

    double current_x_ = 0.0, current_y_ = 0.0, current_yaw_ = 0.0, current_v_ = 0.0;
    bool odom_received_ = false;
    double last_delta_ = 0.0;

    bool goal_active_ = false;
    rclcpp::Time goal_start_time_;
    std::vector<std::pair<double, double>> path_points_;

    Function solver_;
    int n_vars_ = 0;
    int n_constraints_ = 0;
    std::vector<double> lbx_, ubx_, lbg_, ubg_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pred_path_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    void setup_mpc() {
        int nx = 4; // x, y, psi, v
        int nu = 2; // a, delta

        SX X = SX::sym("X", nx, N_ + 1);
        SX U = SX::sym("U", nu, N_);
        SX P = SX::sym("P", nx + N_ * nx);

        SX cost = 0;
        std::vector<SX> g;

        double w_pos = 20.0;
        double w_yaw = 5.0;
        double w_v = 2.0;
        double w_accel = 0.5;
        double w_steer = 1.0;
        double w_dsteer = 5.0;

        // Warunek początkowy
        g.push_back(X(Slice(), 0) - P(Slice(0, nx)));

        for (int k = 0; k < N_; ++k) {
            SX st = X(Slice(), k);
            SX u = U(Slice(), k);

            SX x_k = st(0), y_k = st(1), psi_k = st(2), v_k = st(3);
            SX a_k = u(0), delta_k = u(1);

            // Kinematyka Ackermanna
            SX x_next = x_k + v_k * cos(psi_k) * dt_;
            SX y_next = y_k + v_k * sin(psi_k) * dt_;
            SX psi_next = psi_k + (v_k / L_) * tan(delta_k) * dt_;
            SX v_next = v_k + a_k * dt_;

            SX st_next = vertcat(x_next, y_next, psi_next, v_next);
            g.push_back(X(Slice(), k + 1) - st_next);

            int ref_idx = nx + k * nx;
            SX x_ref = P(ref_idx);
            SX y_ref = P(ref_idx + 1);
            SX psi_ref = P(ref_idx + 2);
            SX v_ref = P(ref_idx + 3);

            cost += w_pos * (pow(x_k - x_ref, 2) + pow(y_k - y_ref, 2));
            cost += w_yaw * pow(sin(psi_k - psi_ref), 2);
            cost += w_v * pow(v_k - v_ref, 2);
            cost += w_accel * pow(a_k, 2);
            cost += w_steer * pow(delta_k, 2);

            if (k == 0) {
                cost += w_dsteer * pow(delta_k - last_delta_, 2);
            } else {
                cost += w_dsteer * pow(delta_k - U(1, k - 1), 2);
            }
        }

        int ref_end_idx = nx + (N_ - 1) * nx;
        cost += w_pos * 2.0 * (pow(X(0, N_) - P(ref_end_idx), 2) + pow(X(1, N_) - P(ref_end_idx + 1), 2));

        SX opt_variables = vertcat(reshape(X, -1, 1), reshape(U, -1, 1));
        SXDict nlp_prob = {{"f", cost}, {"x", opt_variables}, {"g", vertcat(g)}, {"p", P}};

        Dict opts;
        opts["ipopt.max_iter"] = 50;
        opts["ipopt.print_level"] = 0;
        opts["print_time"] = false;
        opts["ipopt.acceptable_tol"] = 1e-4;

        solver_ = nlpsol("solver", "ipopt", nlp_prob, opts);

        n_vars_ = opt_variables.size1();
        n_constraints_ = (N_ + 1) * nx;

        lbx_.clear(); ubx_.clear();
        for (int i = 0; i < N_ + 1; ++i) {
            lbx_.push_back(-inf); lbx_.push_back(-inf); lbx_.push_back(-inf); lbx_.push_back(v_min_);
            ubx_.push_back(inf);  ubx_.push_back(inf);  ubx_.push_back(inf);  ubx_.push_back(v_max_);
        }
        for (int i = 0; i < N_; ++i) {
            lbx_.push_back(-1.0); lbx_.push_back(-delta_max_);
            ubx_.push_back(1.0);  ubx_.push_back(delta_max_);
        }

        lbg_.assign(n_constraints_, 0.0);
        ubg_.assign(n_constraints_, 0.0);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;

        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        current_v_ = std::sqrt(vx * vx + vy * vy);
        odom_received_ = true;
    }

    void plan_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        path_points_.clear();
        for (const auto& p : msg->poses) {
            path_points_.emplace_back(p.pose.position.x, p.pose.position.y);
        }

        if (path_points_.size() >= 2) {
            goal_active_ = true;
            goal_start_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "Odebrano nowa sciezke A* (%zu punktow). Aktywowano failsafe timer (%.1fs).",
                        path_points_.size(), goal_timeout_sec_);
        } else {
            goal_active_ = false;
            stop_robot();
        }
    }

    void control_loop() {
        if (!odom_received_ || !goal_active_ || path_points_.size() < 2) {
            return;
        }

        // 1. FAILSAFE: Sprawdzenie limitu czasu 15 sekund
        double elapsed_sec = (this->now() - goal_start_time_).seconds();
        if (elapsed_sec > goal_timeout_sec_) {
            RCLCPP_WARN(this->get_logger(),
                "FAILSAFE ZADZIALAL: Pojazd nie osiagnal celu w ciagu %.1f s (czas: %.2f s)! Wylaczam naped do czasu podania nastepnego celu.",
                goal_timeout_sec_, elapsed_sec);
            stop_robot();
            goal_active_ = false;
            path_points_.clear();
            return;
        }

        // 2. Sprawdzenie czy osiągnięto cel
        auto [gx, gy] = path_points_.back();
        double dist_to_goal = std::hypot(gx - current_x_, gy - current_y_);
        if (dist_to_goal < goal_tol_) {
            RCLCPP_INFO(this->get_logger(), "Cel osiagniety (odleglosc: %.2f m)! Zatrzymuje robota.", dist_to_goal);
            stop_robot();
            goal_active_ = false;
            path_points_.clear();
            return;
        }

        // 3. Wyszukanie najbliższego punktu na ścieżce
        size_t min_idx = 0;
        double min_dist = 1e9;
        for (size_t i = 0; i < path_points_.size(); ++i) {
            double d = std::hypot(path_points_[i].first - current_x_, path_points_[i].second - current_y_);
            if (d < min_dist) {
                min_dist = d;
                min_idx = i;
            }
        }

        std::vector<double> p_val = {current_x_, current_y_, current_yaw_, current_v_};
        size_t idx = min_idx;
        double v_target = v_max_;
        double step_dist = std::max(v_target * dt_, 0.05);
        double accum_dist = 0.0;

        for (int k = 0; k < N_; ++k) {
            double target_dist = (k + 1) * step_dist;
            while (idx < path_points_.size() - 1) {
                double seg_d = std::hypot(path_points_[idx + 1].first - path_points_[idx].first,
                                          path_points_[idx + 1].second - path_points_[idx].second);
                if (accum_dist + seg_d >= target_dist) break;
                accum_dist += seg_d;
                idx++;
            }

            auto [pt_x, pt_y] = path_points_[std::min(idx, path_points_.size() - 1)];
            double ref_yaw = current_yaw_;
            if (idx < path_points_.size() - 1) {
                ref_yaw = std::atan2(path_points_[idx + 1].second - pt_y, path_points_[idx + 1].first - pt_x);
            }

            p_val.push_back(pt_x);
            p_val.push_back(pt_y);
            p_val.push_back(ref_yaw);
            p_val.push_back(v_target);
        }

        try {
            std::vector<double> x0(n_vars_, 0.0);
            DMDict arg = {
                {"x0", x0},
                {"lbx", lbx_},
                {"ubx", ubx_},
                {"lbg", lbg_},
                {"ubg", ubg_},
                {"p", p_val}
            };
            DMDict res = solver_(arg);
            std::vector<double> sol = std::vector<double>(res.at("x"));

            int u_start_idx = (N_ + 1) * 4;
            double a_cmd = sol[u_start_idx];
            double delta_cmd = sol[u_start_idx + 1];

            double v_cmd = std::clamp(current_v_ + a_cmd * dt_, v_min_, v_max_);
            last_delta_ = delta_cmd;

            auto cmd = geometry_msgs::msg::Twist();
            cmd.linear.x = v_cmd;
            cmd.angular.z = delta_cmd;
            cmd_pub_->publish(cmd);

            // Publikacja przewidywanej trajektorii MPC
            auto pred_path = nav_msgs::msg::Path();
            pred_path.header.stamp = this->now();
            pred_path.header.frame_id = "world";

            for (int k = 0; k <= N_; ++k) {
                auto ps = geometry_msgs::msg::PoseStamped();
                ps.header = pred_path.header;
                ps.pose.position.x = sol[k * 4];
                ps.pose.position.y = sol[k * 4 + 1];
                ps.pose.position.z = 0.0;
                pred_path.poses.push_back(ps);
            }
            pred_path_pub_->publish(pred_path);

        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Blad obliczen CasADi MPC: %s", e.what());
            stop_robot();
        }
    }

    void stop_robot() {
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CasadiMpcNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
