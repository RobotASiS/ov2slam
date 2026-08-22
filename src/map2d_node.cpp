#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include "std_msgs/msg/float32.hpp"

class MapperNode : public rclcpp::Node {
public:
    MapperNode() : Node("mapper2d") {
        pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud", 10, std::bind(&MapperNode::point_cloud_callback, this, std::placeholders::_1));

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&MapperNode::odom_callback, this, std::placeholders::_1));

        
        scale_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/slam_scale", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
                current_scale = msg->data;
            });
        map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/map", 10);
        grid_map.assign(width_map * height_map, 0);
        goal_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal",10);
        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    }

    void draw_gui() {
        ImGui::Begin("map");
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 p = ImGui::GetCursorScreenPos();

        for (int y = 0; y < height_map; y++) {
            for (int x = 0; x < width_map; x++) {
                int index = (y * width_map) + x;
                int hits = grid_map[index];
                ImU32 color;

                if (hits > 0) {
                    int redness = hits * 10;
                    color = IM_COL32(std::min(redness, 255), 0, 0, 255);
                } else {
                    color = IM_COL32(255, 255, 255, 255);
                }

                draw_list->AddRectFilled(
                    ImVec2(p.x + (x * cell_draw_size), p.y + (y * cell_draw_size)),
                    ImVec2(p.x + ((x + 1) * cell_draw_size), p.y + ((y + 1) * cell_draw_size)),
                    color);
            }
        }
        
        float rx = pose_x;
        float ry = pose_y;
        if (is_calibrated) {
            float c = std::cos(-calib_yaw);
            float s = std::sin(-calib_yaw);
            float tx = rx * c - ry * s;
            float ty = rx * s + ry * c;
            rx = tx;
            ry = ty;
        }

        int robot_grid_x = static_cast<int>((rx + (map_size / 2.0f)) / resolution);
        int robot_grid_y = static_cast<int>((ry + (map_size / 2.0f)) / resolution);

        if (robot_grid_x >= 0 && robot_grid_x < width_map && robot_grid_y >= 0 && robot_grid_y < height_map) {
            draw_list->AddCircleFilled(
                ImVec2(p.x + robot_grid_x * cell_draw_size + (cell_draw_size / 2.0f), p.y + robot_grid_y * cell_draw_size + (cell_draw_size / 2.0f)),
                cell_draw_size * 0.8f,
                IM_COL32(0, 255, 0, 255));
        }

        ImGui::InvisibleButton(ImVec2(width_map * cell_draw_size, height_map * cell_draw_size));
        if(ImGui::IsItemHovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left) {
          ImVec2 mouse_pos = ImGui::GetIO().MousePos;
          int clicked_grid_x = static_cast<int>((mouse_pos.x - p.x) / cell_draw_size);
          int clicked_grid_y = static_cast<int>((mouse_pos.y - p.y) / cell_draw_size);

          if (clicked_grid_x >= 0 && clicked_grid_x < width_map && clicked_grid_y >= 0 && clicked_grid_y < height_map) {
              int clicked_index = (clicked_grid_y * width_map) + clicked_grid_x;
            }
        ImGui::End();
    }

private:
    void publish_goal(int GridCell ){
      auto msg = geometry_msgs::msg::PoseStamped();

      msg.header.stamp = this->now();
      msg.header.frame_id = "world";
      x = GridCell%this->rows;
      y = GridCell/this->rows;
      msg.pose.position.x = x*resolution;
      msg.pose.position.y = y*resolution;
      msg.pose.position.z = 0;
      msg.pose.orientation.x = 0.0
      msg.pose.orientation.y = 0.0
      msg.pose.orientation.z = 0.0
      msg.pose.orientation.w = 1.0
      goal_pub->publish(msg);
    }
    void NavMap(){
      auto msg = nav_msgs::msg::OccupancyGrid();

      msg.header.stamp = this->now();
      msg.header.frame_id = "world";
      
      msg.info.width = this->width_map;
      msg.info.height = this->height_map;
      msg.info.resolution = this->resolution;

      double origin_x = - (msg.info.width * msg.info.resolution) / 2.0;
      double origin_y = - (msg.info.height * msg.info.resolution) / 2.0;

      msg.info.origin.position.x = origin_x;
      msg.info.origin.position.y = origin_y;
      msg.info.origin.position.z = 0.0;

      msg.info.origin.orientation.x = 0.0;
      msg.info.origin.orientation.y = 0.0;
      msg.info.origin.orientation.z = 0.0;
      msg.info.origin.orientation.w = 1.0;
      
      std::vector<int8_t> grid_data = grid_map;
      for (size_t i = 0; i < grid_data.size(); i++){
        if (grid_data[i] != 0){
          grid_data[i] = 100;
        }
      }
      msg.data = std::move(grid_data);
      map_pub->publish(msg);
    }
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg, *pcl_cloud);
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    
        if (is_calibrated) {
            transform.rotate(Eigen::AngleAxisf(-calib_yaw, Eigen::Vector3f::UnitZ()));
        }
        
        transform.rotate(Eigen::AngleAxisf(-M_PI/2.0, Eigen::Vector3f::UnitZ()));
        transform.rotate(Eigen::AngleAxisf(-M_PI/2.0, Eigen::Vector3f::UnitX()));

        auto transformed_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::transformPointCloud(*pcl_cloud, *transformed_cloud, transform);
        
        // Filtracja szumow (Statistical Outlier Removal)
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(transformed_cloud);
        sor.setMeanK(20);             
        sor.setStddevMulThresh(1.0);  
        sor.filter(*filtered_cloud);

        mapping(filtered_cloud);
        NavMap();
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        pose_x = msg->pose.pose.position.x;
        pose_y = msg->pose.pose.position.y; 
        
        if (!has_initial_pose) {
            initial_pose_x = pose_x;
            initial_pose_y = pose_y;
            has_initial_pose = true;
        }

        if (!is_calibrated) {
            float dx = pose_x - initial_pose_x;
            float dy = pose_y - initial_pose_y;
            float dist = std::sqrt(dx*dx + dy*dy);
            if (dist > 0.2f) {
                calib_yaw = std::atan2(dy, dx);
                is_calibrated = true;
                RCLCPP_INFO(this->get_logger(), "Trajektoria wyrownana! Kat rotacji: %f rad", calib_yaw);
            }
        }
    }

   

    void mapping(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        std::fill(grid_map.begin(), grid_map.end(), 0);
        
        std::vector<int> point_counts(width_map * height_map, 0);

        for (auto& point : cloud->points) {
            float px = point.x * current_scale;
            float py = point.y * current_scale;
            float pz = point.z * current_scale;
            
            if (pz < -0.02 || pz > 0.3) {
                continue;
            }

            int grid_x = static_cast<int>((px + (map_size / 2.0)) / resolution);
            int grid_y = static_cast<int>((py + (map_size / 2.0)) / resolution);

            if (grid_x < 0 || grid_x >= width_map || grid_y < 0 || grid_y >= height_map) {
                continue;
            }

            int index = (grid_y * width_map) + grid_x;
            point_counts[index]++;
        }

        // Mapowanie do wektora z uwzglednieniem minimalnej liczby punktow per komorka
        for (size_t i = 0; i < point_counts.size(); ++i) {
            if (point_counts[i] >= 3) { // Prog - wymagamy np. 3 punktow, by uznac za przeszkode
                grid_map[i] = point_counts[i]; // Utrzymujemy ilosc, zeby miec gladki kolor
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr scale_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    float slam_height_est = -1.0f;
    
    // Zmienne kalibracyjne
    bool is_calibrated = false;
    bool has_initial_pose = false;
    float initial_pose_x = 0.0f;
    float initial_pose_y = 0.0f;
    float calib_yaw = 0.0f;

    // Zwiekszona rozdzielczosc mapy
    const uint32_t width_map = 250;
    const uint32_t height_map = 250;
    const float map_size = 4.0f; 
    const float resolution = map_size / width_map;
    const float cell_draw_size = 3.0f;
    const int rows = map_size / resolution;
    
    std::vector<int8_t> grid_map;
    float pose_x = 0.0f;
    float pose_y = 0.0f;
    float current_scale = 1.0f;
    const float cam_height = 0.092f; 
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapperNode>();

    if (!glfwInit()) return -1;
    GLFWwindow* window = glfwCreateWindow(800, 800, "OV2SLAM MAP", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        rclcpp::spin_some(node);
        node->draw_gui();

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    rclcpp::shutdown();
    return 0;
}
