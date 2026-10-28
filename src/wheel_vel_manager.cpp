#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <cstring>
#include <chrono>
#include <memory>

class WheelVelManager : public rclcpp::Node {
public:
    WheelVelManager() : Node("wheel_vel_manager"), i2c_file_(-1), v_linear_(0.0f), psi_(0.0f) {
        this->declare_parameter("track_width", 0.25);
        this->declare_parameter("wheelbase", 0.25);
        this->declare_parameter("i2c_device", "/dev/i2c-1");
        this->declare_parameter("i2c_address", 0x68);
        this->declare_parameter("wheel_diameter", 0.0705);
        this->declare_parameter("ticks_per_rev", 0.0);
        this->declare_parameter("servo_center_deg", 90.0);
        this->declare_parameter("timeout_sec", 0.5);

        track_width_ = this->get_parameter("track_width").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        i2c_device_ = this->get_parameter("i2c_device").as_string();
        i2c_address_ = this->get_parameter("i2c_address").as_int();
        wheel_diameter_ = this->get_parameter("wheel_diameter").as_double();
        ticks_per_rev_ = this->get_parameter("ticks_per_rev").as_double();
        servo_center_deg_ = this->get_parameter("servo_center_deg").as_double();
        timeout_sec_ = this->get_parameter("timeout_sec").as_double();

        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&WheelVelManager::vel_callback, this, std::placeholders::_1));

        if ((i2c_file_ = open(i2c_device_.c_str(), O_RDWR)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Nie mozna otworzyc %s", i2c_device_.c_str());
        } else {
            if (ioctl(i2c_file_, I2C_SLAVE, i2c_address_) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Nie mozna ustawic adresu I2C 0x%02X", i2c_address_);
            }
        }

        if (ticks_per_rev_ <= 0.0) {
            RCLCPP_ERROR(this->get_logger(), "ticks_per_rev nie ustawione - kola nie beda jechac");
        }

        last_cmd_time_ = this->now();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&WheelVelManager::timer_callback, this));
    }

    ~WheelVelManager() {
        if (i2c_file_ >= 0) {
            close(i2c_file_);
        }
    }

private:
    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        v_linear_ = static_cast<float>(msg->linear.x);
        psi_ = static_cast<float>(msg->angular.z);
        last_cmd_time_ = this->now();
    }

    void timer_callback() {
        float vl = 0.0f;
        float vr = 0.0f;
        float psi_deg = static_cast<float>(servo_center_deg_);

        if ((this->now() - last_cmd_time_).seconds() <= timeout_sec_) {
            float tan_psi = std::tan(psi_);
            float factor = (static_cast<float>(track_width_) / (2.0f * static_cast<float>(wheelbase_))) * tan_psi;
            vl = v_linear_ * (1.0f - factor);
            vr = v_linear_ * (1.0f + factor);
            psi_deg = static_cast<float>(psi_ * 180.0f / M_PI) + static_cast<float>(servo_center_deg_);
        }

        if (i2c_file_ >= 0) {
            // Ramka ESP (I2C_data_frame_RPI): 3x int32 LE, bez bajtu rejestru
            // [predkosc prawego kola, predkosc lewego kola] w tickach enkodera/s, [serwo] w stopniach (90 = srodek)
            float ticks_per_m = static_cast<float>(ticks_per_rev_ / (M_PI * wheel_diameter_));
            int32_t frame[3] = {
                static_cast<int32_t>(std::lround(vr * ticks_per_m)),
                static_cast<int32_t>(std::lround(vl * ticks_per_m)),
                static_cast<int32_t>(std::lround(psi_deg))
            };

            if (write(i2c_file_, frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Blad zapisu I2C");
            }
        }
    }

    int i2c_file_;
    std::string i2c_device_;
    int i2c_address_;
    double wheel_diameter_;
    double ticks_per_rev_;
    double track_width_;
    double wheelbase_;
    double servo_center_deg_;
    double timeout_sec_;

    float v_linear_;
    float psi_;
    rclcpp::Time last_cmd_time_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelVelManager>());
    rclcpp::shutdown();
    return 0;
}
