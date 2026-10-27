#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <mutex>

using namespace std::chrono_literals;

// RPI ---> ESP (12 bajtów packed)
struct __attribute__((packed)) I2C_data_frame_RPI {
    int32_t right_motor_speed; // setpoint w impulsach enkoderów / s
    int32_t left_motor_speed;  // setpoint w impulsach enkoderów / s
    int32_t servo_pos;         // kąt serwa 0..180 (90 = wprost)
};

// RPI <--- ESP (16 bajtów packed)
struct __attribute__((packed)) I2C_data_frame_ESP {
    int64_t right_encoder_pos; // sumaryczna pozycja enkodera
    int64_t left_encoder_pos;
};

class Esp32I2cBridgeNode : public rclcpp::Node {
public:
    Esp32I2cBridgeNode() : Node("esp32_i2c_bridge_node"), i2c_fd_(-1) {
        // Deklaracja parametrów konfiguracyjnych
        this->declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
        this->declare_parameter<int>("i2c_address", 0x68);
        this->declare_parameter<double>("wheel_radius", 0.03525); // [m] (średnica 70.5mm z robot_data.h)
        this->declare_parameter<double>("ticks_per_rev", 660.0);   // impulsy na obrót koła
        this->declare_parameter<double>("wheelbase", 0.20);        // [m] rozstaw osi L
        this->declare_parameter<double>("track_width", 0.15);      // [m] rozstaw kół W
        this->declare_parameter<int>("servo_center", 90);          // kąt neutralny [st.] (środek pomiędzy 50 a 130)
        this->declare_parameter<int>("servo_min", 50);             // min bezpieczny kąt [st.] (max prawo: 50 st.)
        this->declare_parameter<int>("servo_max", 130);            // max bezpieczny kąt [st.] (max lewo: 130 st.)
        this->declare_parameter<double>("servo_deg_per_rad", 57.3);// współczynnik przeliczenia rad -> st. (180/pi)
        this->declare_parameter<bool>("invert_steering", false);
        this->declare_parameter<bool>("invert_left_motor", false);
        this->declare_parameter<int>("cmd_timeout_ms", 500);       // watchdog zatrzymania silników

        i2c_device_ = this->get_parameter("i2c_device").as_string();
        i2c_address_ = this->get_parameter("i2c_address").as_int();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        ticks_per_rev_ = this->get_parameter("ticks_per_rev").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        track_width_ = this->get_parameter("track_width").as_double();
        servo_center_ = this->get_parameter("servo_center").as_int();
        servo_min_ = this->get_parameter("servo_min").as_int();
        servo_max_ = this->get_parameter("servo_max").as_int();
        servo_deg_per_rad_ = this->get_parameter("servo_deg_per_rad").as_double();
        invert_steering_ = this->get_parameter("invert_steering").as_bool();
        invert_left_motor_ = this->get_parameter("invert_left_motor").as_bool();
        cmd_timeout_ms_ = this->get_parameter("cmd_timeout_ms").as_int();

        init_i2c();

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&Esp32I2cBridgeNode::cmd_vel_callback, this, std::placeholders::_1)
        );

        encoder_pub_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("/esp32/encoders", 10);

        // Timer pętli komunikacyjnej I2C (20 Hz = 50ms)
        last_cmd_time_ = this->get_clock()->now();
        current_frame_rpi_ = {0, 0, static_cast<int32_t>(servo_center_)};

        comm_timer_ = this->create_wall_timer(
            50ms, std::bind(&Esp32I2cBridgeNode::i2c_comm_loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "Węzeł esp32_i2c_bridge_node uruchomiony (Adres I2C: 0x%02X)", i2c_address_);
    }

    ~Esp32I2cBridgeNode() {
        // Bezpieczne zatrzymanie silników przy wyłączeniu
        if (i2c_fd_ >= 0) {
            I2C_data_frame_RPI stop_frame{0, 0, static_cast<int32_t>(servo_center_)};
            write(i2c_fd_, &stop_frame, sizeof(stop_frame));
            close(i2c_fd_);
        }
    }

private:
    std::string i2c_device_;
    int i2c_address_;
    double wheel_radius_;
    double ticks_per_rev_;
    double wheelbase_;
    double track_width_;
    int servo_center_;
    int servo_min_;
    int servo_max_;
    double servo_deg_per_rad_;
    bool invert_steering_;
    bool invert_left_motor_;
    int cmd_timeout_ms_;

    int i2c_fd_;
    std::mutex i2c_mutex_;
    rclcpp::Time last_cmd_time_;
    I2C_data_frame_RPI current_frame_rpi_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr encoder_pub_;
    rclcpp::TimerBase::SharedPtr comm_timer_;

    void init_i2c() {
        i2c_fd_ = open(i2c_device_.c_str(), O_RDWR);
        if (i2c_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Nie można otworzyć magistrali I2C: %s", i2c_device_.c_str());
            return;
        }
        if (ioctl(i2c_fd_, I2C_SLAVE, i2c_address_) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Nie można ustawić adresu I2C SLAVE: 0x%02X", i2c_address_);
            close(i2c_fd_);
            i2c_fd_ = -1;
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Połączono z ESP32 przez I2C (%s @ 0x%02X)", i2c_device_.c_str(), i2c_address_);
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(i2c_mutex_);
        last_cmd_time_ = this->get_clock()->now();

        double v = msg->linear.x;       // zadana prędkość liniowa [m/s]
        double delta = msg->angular.z;  // zadany kąt skrętu [rad] lub prędkość kątowa

        // Obliczenie kąta serwa
        double steer_deg = (invert_steering_ ? -1.0 : 1.0) * delta * servo_deg_per_rad_;
        int target_servo = std::clamp(static_cast<int>(std::round(servo_center_ + steer_deg)), servo_min_, servo_max_);

        // Kinematyka Ackermanna: różnicowanie prędkości lewego i prawego koła
        double v_l = v;
        double v_r = v;
        if (std::abs(wheelbase_) > 1e-3 && std::abs(delta) > 1e-4) {
            double tan_delta = std::tan(delta);
            v_l = v * (1.0 - (track_width_ / (2.0 * wheelbase_)) * tan_delta);
            v_r = v * (1.0 + (track_width_ / (2.0 * wheelbase_)) * tan_delta);
        }

        // Przeliczenie m/s -> impulsy enkoderów / s
        // obroty/s = v / (2 * pi * R)
        // ticks/s = (v / (2 * pi * R)) * ticks_per_rev
        double circumference = 2.0 * M_PI * wheel_radius_;
        int32_t left_ticks_per_sec = 0;
        int32_t right_ticks_per_sec = 0;

        if (circumference > 1e-4) {
            left_ticks_per_sec = static_cast<int32_t>(std::round((v_l / circumference) * ticks_per_rev_));
            right_ticks_per_sec = static_cast<int32_t>(std::round((v_r / circumference) * ticks_per_rev_));
        }

        current_frame_rpi_.left_motor_speed = (invert_left_motor_ ? -1 : 1) * left_ticks_per_sec;
        current_frame_rpi_.right_motor_speed = right_ticks_per_sec;
        current_frame_rpi_.servo_pos = target_servo;
    }

    void i2c_comm_loop() {
        if (i2c_fd_ < 0) {
            init_i2c();
            if (i2c_fd_ < 0) return;
        }

        std::lock_guard<std::mutex> lock(i2c_mutex_);

        // Watchdog bezpieczeństwa: zatrzymanie przy braku komend
        auto now = this->get_clock()->now();
        double elapsed_ms = (now - last_cmd_time_).nanoseconds() / 1e6;
        if (elapsed_ms > cmd_timeout_ms_) {
            current_frame_rpi_.left_motor_speed = 0;
            current_frame_rpi_.right_motor_speed = 0;
            current_frame_rpi_.servo_pos = servo_center_;
        }

        // 1. Zapis ramki sterującej RPI ---> ESP (12 bajtów)
        ssize_t bytes_written = write(i2c_fd_, &current_frame_rpi_, sizeof(current_frame_rpi_));
        if (bytes_written != sizeof(current_frame_rpi_)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Błąd zapisu ramki I2C do ESP32 (wysłano %zd/%zu bajtów)", bytes_written, sizeof(current_frame_rpi_));
        }

        // 2. Odczyt danych enkoderów ESP ---> RPI (rejestr 0x01, następnie 16 bajtów)
        uint8_t reg = 0x01;
        if (write(i2c_fd_, &reg, 1) == 1) {
            I2C_data_frame_ESP esp_frame{0, 0};
            ssize_t bytes_read = read(i2c_fd_, &esp_frame, sizeof(esp_frame));
            if (bytes_read == sizeof(esp_frame)) {
                auto enc_msg = std_msgs::msg::Int64MultiArray();
                enc_msg.data = {(invert_left_motor_ ? -1 : 1) * esp_frame.left_encoder_pos, esp_frame.right_encoder_pos};
                encoder_pub_->publish(enc_msg);
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Błąd odczytu ramki enkoderów z ESP32 (odebrano %zd/%zu bajtów)", bytes_read, sizeof(esp_frame));
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Esp32I2cBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
