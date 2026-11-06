#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cmath>
// #include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "nav_msgs/msg/path.hpp"

class MapperNode : public rclcpp::Node {
public:
    MapperNode() : Node("mapper2d") {
        pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud", 10, std::bind(&MapperNode::point_cloud_callback, this, std::placeholders::_1));

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&MapperNode::odom_callback, this, std::placeholders::_1));

        map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/map", 10);
        grid_map.assign(width_map * height_map, 0);
        goal_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal",10);
        path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&MapperNode::path_callback, this, std::placeholders::_1));
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
                    ImVec2(p.x + (x * cell_draw_size), p.y + ((height_map - 1 - y) * cell_draw_size)),
                    ImVec2(p.x + ((x + 1) * cell_draw_size), p.y + ((height_map - y) * cell_draw_size)),
                    color);

            }
        }
        
        float rx = pose_x;
        float ry = pose_y;

        int robot_grid_x = static_cast<int>((rx + (map_size / 2.0f)) / resolution);
        int robot_grid_y = static_cast<int>((ry + (map_size / 2.0f)) / resolution);

        if (robot_grid_x >= 0 && robot_grid_x < width_map && robot_grid_y >= 0 && robot_grid_y < height_map) {
            ImVec2 center(p.x + robot_grid_x * cell_draw_size + (cell_draw_size / 2.0f),
                          p.y + (height_map - 1 - robot_grid_y) * cell_draw_size + (cell_draw_size / 2.0f));
            draw_list->AddCircleFilled(center, 6.0f, IM_COL32(0, 255, 0, 255));

            // Czerwona kreska wskazująca przód (orientację yaw) robota
            float heading_length = 30.0f;
            ImVec2 heading_end(center.x + std::cos(pose_yaw) * heading_length,
                               center.y - std::sin(pose_yaw) * heading_length);
            draw_list->AddLine(center, heading_end, IM_COL32(255, 0, 0, 255), 2.5f);
        }

        ImGui::InvisibleButton("map", ImVec2(width_map * cell_draw_size, height_map * cell_draw_size));
        if(ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
          ImVec2 mouse_pos = ImGui::GetIO().MousePos;
          int clicked_grid_x = static_cast<int>((mouse_pos.x - p.x) / cell_draw_size);
          int clicked_grid_y = height_map - 1 - static_cast<int>((mouse_pos.y - p.y) / cell_draw_size);
          if (clicked_grid_x >= 0 && clicked_grid_x < width_map && clicked_grid_y >= 0 && clicked_grid_y < height_map) {
              int clicked_index = (clicked_grid_y * width_map) + clicked_grid_x;
              this->publish_goal(clicked_index);
          }
        }
        for (const auto & pose_stamped : this->path.poses) {
            ImVec2 screen_pos = this->pose_to_grid(pose_stamped, p);
            draw_list->AddCircleFilled(screen_pos, 2.5f, IM_COL32(0, 200, 255, 255)); 
        }
        ImGui::Text("Pozycja robota: X=%.2f m, Y=%.2f m, Yaw=%.1f deg", pose_x, pose_y, pose_yaw * 180.0f / M_PI);
        if (floor_valid) {
            ImGui::Text("Podloga: pochylenie X=%.2f deg, Y=%.2f deg, c=%.3f m",
                        std::atan(floor_a) * 180.0f / M_PI, std::atan(floor_b) * 180.0f / M_PI, floor_c);
        } else {
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.0f, 1.0f), "Podloga: brak estymacji (stala cam_height)");
        }
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Czerwona linia = przod (+X w prawo, +Y w gore na ekranie)");
        ImGui::End();
    }

private:
    void publish_goal(int GridCell ){
      auto msg = geometry_msgs::msg::PoseStamped();

      msg.header.stamp = this->now();
      msg.header.frame_id = "world";
      int gx = GridCell % width_map;
      int gy = GridCell / width_map;

      msg.pose.position.x = (gx * resolution) - (map_size / 2.0f) + (resolution / 2.0f);
      msg.pose.position.y = (gy * resolution) - (map_size / 2.0f) + (resolution / 2.0f);
      msg.pose.position.z = 0.0;

      // Planner uwzglednia kat w celu, wiec robot ma dojechac przodem w kierunku klikniecia
      double goal_yaw = std::atan2(msg.pose.position.y - pose_y, msg.pose.position.x - pose_x);
      msg.pose.orientation.x = 0.0;
      msg.pose.orientation.y = 0.0;
      msg.pose.orientation.z = std::sin(goal_yaw / 2.0);
      msg.pose.orientation.w = std::cos(goal_yaw / 2.0);
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
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg){
      this->path = *msg;
    }
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Uklad kamery (Z przod, X prawo, Y dol) -> ROS (X przod, Y lewo, Z gora)
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(-M_PI/2.0, Eigen::Vector3f::UnitZ()));
        transform.rotate(Eigen::AngleAxisf(-M_PI/2.0, Eigen::Vector3f::UnitX()));

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*pcl_cloud, *transformed_cloud, transform);

        // Usuwamy NaN, Inf oraz punkty (0,0,0) - tak SLAM oznacza usuniete landmarki
        pcl::PointCloud<pcl::PointXYZ>::Ptr clean_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        clean_cloud->points.reserve(transformed_cloud->points.size());
        for (const auto& pt : transformed_cloud->points) {
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
                if (std::abs(pt.x) > 0.01f || std::abs(pt.y) > 0.01f || std::abs(pt.z) > 0.01f) {
                    clean_cloud->points.push_back(pt);
                }
            }
        }

        // Filtracja szumow (Statistical Outlier Removal) - na razie wylaczona, wycinala za duzo punktow
        // if (clean_cloud->points.size() >= 20) {
        //     pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        //     pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        //     sor.setInputCloud(clean_cloud);
        //     sor.setMeanK(20);
        //     sor.setStddevMulThresh(1.0);
        //     sor.filter(*filtered_cloud);
        //     clean_cloud = filtered_cloud;
        // }

        estimate_floor(clean_cloud);
        mapping(clean_cloud);
        NavMap();
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        pose_x = msg->pose.pose.position.x;
        pose_y = msg->pose.pose.position.y; 
        const auto & q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        pose_yaw = std::atan2(siny_cosp, cosy_cosp);
    }

   
    ImVec2 pose_to_grid(const geometry_msgs::msg::PoseStamped & pose_stamped, const ImVec2 & p) {
        float px = pose_stamped.pose.position.x;
        float py = pose_stamped.pose.position.y;

        int grid_x = static_cast<int>((px + (map_size / 2.0f)) / resolution);
        int grid_y = static_cast<int>((py + (map_size / 2.0f)) / resolution);

        return ImVec2(p.x + grid_x * cell_draw_size + (cell_draw_size / 2.0f),
                      p.y + (height_map - 1 - grid_y) * cell_draw_size + (cell_draw_size / 2.0f));
    }

    // Kamera jest lekko pochylona wzgledem podlogi, a uklad world SLAMa to pierwsza poza kamery.
    // Przez to podloga "wznosi sie" z odlegloscia i dalekie punkty podlogi wpadaja w pas przeszkod.
    // Dopasowujemy plaszczyzne podlogi z = a*x + b*y + c (RANSAC) i wysokosc liczymy wzgledem niej.
    void estimate_floor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        // Kandydaci na podloge: punkty w zasiegu i nisko (ponizej kamery, z zapasem na pochylenie)
        pcl::PointCloud<pcl::PointXYZ>::Ptr candidates(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto& pt : cloud->points) {
            float dx = pt.x - pose_x;
            float dy = pt.y - pose_y;
            if (dx * dx + dy * dy > floor_search_radius * floor_search_radius) continue;
            if (pt.z > floor_max_z) continue;
            candidates->points.push_back(pt);
        }
        if (static_cast<int>(candidates->points.size()) < floor_min_inliers) return;
        candidates->width = candidates->points.size();
        candidates->height = 1;

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
        seg.setEpsAngle(floor_max_tilt);
        seg.setDistanceThreshold(floor_ransac_dist);
        seg.setMaxIterations(200);
        seg.setInputCloud(candidates);

        pcl::PointIndices inliers;
        pcl::ModelCoefficients coeffs;
        seg.segment(inliers, coeffs);
        if (static_cast<int>(inliers.indices.size()) < floor_min_inliers) return;

        // n_x*x + n_y*y + n_z*z + d = 0  ->  z = a*x + b*y + c
        float nx = coeffs.values[0], ny = coeffs.values[1], nz = coeffs.values[2], d = coeffs.values[3];
        if (std::abs(nz) < 1e-3f) return;
        float a = -nx / nz;
        float b = -ny / nz;
        float c = -d / nz;

        // Sanity: podloga musi byc mniej wiecej na wysokosci -cam_height (inaczej to np. blat stolu)
        if (std::abs(c + cam_height) > floor_max_offset) return;

        if (!floor_valid) {
            floor_a = a; floor_b = b; floor_c = c;
            floor_valid = true;
        } else {
            floor_a += floor_alpha * (a - floor_a);
            floor_b += floor_alpha * (b - floor_b);
            floor_c += floor_alpha * (c - floor_c);
        }
    }

    float height_above_floor(const pcl::PointXYZ& pt) const {
        if (floor_valid) {
            return pt.z - (floor_a * pt.x + floor_b * pt.y + floor_c);
        }
        return pt.z + cam_height;
    }

    void mapping(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        std::fill(grid_map.begin(), grid_map.end(), 0);
        
        std::vector<int> point_counts(width_map * height_map, 0);

        for (auto& point : cloud->points) {
            // Wysokosc nad dopasowana plaszczyzna podlogi
            float z_floor = height_above_floor(point);
            if (z_floor < min_obstacle_height || z_floor > max_obstacle_height) {
                continue;
            }

            int grid_x = static_cast<int>((point.x + (map_size / 2.0)) / resolution);
            int grid_y = static_cast<int>((point.y + (map_size / 2.0)) / resolution);

            if (grid_x < 0 || grid_x >= width_map || grid_y < 0 || grid_y >= height_map) {
                continue;
            }

            int index = (grid_y * width_map) + grid_x;
            point_counts[index]++;
        }

        // Komorki, ktore przeszly prog liczby punktow
        std::vector<uint8_t> occupied(width_map * height_map, 0);
        for (size_t i = 0; i < point_counts.size(); ++i) {
            if (point_counts[i] >= min_points_per_cell) {
                occupied[i] = 1;
            }
        }

        // Filtr samotnych komorek: zajeta komorka bez zajetych sasiadow (8-sasiedztwo)
        // to zwykle szum z SLAMa - wywalamy, chyba ze ma bardzo duzo punktow (np. cienka noga stolu)
        for (int y = 0; y < height_map; ++y) {
            for (int x = 0; x < width_map; ++x) {
                int index = (y * width_map) + x;
                if (!occupied[index]) {
                    continue;
                }
                int neighbors = 0;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx < 0 || nx >= width_map || ny < 0 || ny >= height_map) continue;
                        neighbors += occupied[(ny * width_map) + nx];
                    }
                }
                if (neighbors >= min_neighbors || point_counts[index] >= strong_cell_points) {
                    grid_map[index] = static_cast<int8_t>(std::min(point_counts[index], 127)); // int8_t max
                }
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;

    // Mapa 4 x 4 m, komorka 2 cm
    const int width_map = 200;
    const int height_map = 200;
    const float map_size = 4.0f;
    const float resolution = map_size / width_map;
    const float cell_draw_size = 3.5f;

    // Przeszkody liczone od podlogi [m]
    const float cam_height = 0.092f;
    const float min_obstacle_height = 0.03f;
    const float max_obstacle_height = 0.50f;

    // Estymacja plaszczyzny podlogi (RANSAC)
    const float floor_search_radius = 2.0f;               // [m] wokol robota
    const float floor_max_z = 0.10f;                      // [m] kandydaci nizej niz to (uklad kamery)
    const float floor_max_tilt = 15.0f * M_PI / 180.0f;   // max odchylenie normalnej od pionu
    const float floor_ransac_dist = 0.02f;                // [m] prog inliera
    const int floor_min_inliers = 30;
    const float floor_max_offset = 0.06f;                 // [m] max roznica c od -cam_height
    const float floor_alpha = 0.2f;                       // filtr dolnoprzepustowy wspolczynnikow
    bool floor_valid = false;
    float floor_a = 0.0f, floor_b = 0.0f, floor_c = 0.0f;

    // Filtracja szumu w siatce
    const int min_points_per_cell = 3;   // min. punktow zeby komorka byla zajeta
    const int min_neighbors = 1;         // min. zajetych sasiadow zeby komorka przetrwala
    const int strong_cell_points = 10;   // tyle punktow = zostaje nawet jak samotna

    std::vector<int8_t> grid_map;
    nav_msgs::msg::Path path;
    float pose_x = 0.0f;
    float pose_y = 0.0f;
    float pose_yaw = 0.0f;
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

    while (rclcpp::ok() && !glfwWindowShouldClose(window)) {
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

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
