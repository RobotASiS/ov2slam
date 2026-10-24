import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ov2slam')

    # Ścieżka do konfiguracji EKF
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    
    # Ścieżka do parametrów OV2SLAM (Stereo)
    ov2slam_config_path = os.path.join(pkg_share, 'parameters_files', 'accurate', 'euroc', 'wavesshare_cam.yaml')
    
    # Ścieżka do konfiguracji RViz
    rviz_config_path = os.path.join(pkg_share, 'ov2slam_visualization.rviz') 

    # Ścieżka do parametrów plannera Nav2
    planner_config_path = os.path.join(pkg_share, 'config', 'planner_server.yaml')

    return LaunchDescription([
        # 0. Węzeł kamery stereo (V4L2)
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',   
                'image_size': [2560, 720],       
                'pixel_format': 'YUYV'
            }]
        ),

        # 1. Węzeł rozdzielający obraz na left i right
        Node(
            package='ov2slam',
            executable='stereo_publisher.py',
            name='stereo_publisher',
            output='screen'
        ),

        # 2. Główny węzeł OV2SLAM (Stereo)
        Node(
            package='ov2slam',
            executable='ov2slam_node',
            name='ov2slam_node',
            output='screen',
            arguments=[ov2slam_config_path]
        ),

        # 3. Węzeł dodający kowariancję do pozy VO (konwersja układu współrzędnych)
        Node(
            package='ov2slam',
            executable='pose_cov_adder_node',
            name='pose_cov_adder_node',
            output='screen'
        ),

        # 4. Węzeł filtra EKF (robot_localization)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),

        # 5. Węzeł mapy 2D (ImGui)
        Node(
            package='ov2slam',
            executable='map2d_node',
            name='map2d_node',
            output='screen'
        ),

        # 6. Globalny Planner Nav2 (A*)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_config_path]
        ),

        # Menedżer cyklu życia dla Nav2 planner_server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planner',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['planner_server']
            }]
        ),

        # Klient akcji zapytania o ścieżkę A* po otrzymaniu /goal
        Node(
            package='ov2slam',
            executable='path_planner_node',
            name='path_planner',
            output='screen',
        ),

        # 7. Regulator CasADi MPC (śledzenie ścieżki /plan z A*)
        Node(
            package='ov2slam',
            executable='casadi_mpc_node',
            name='casadi_mpc_node',
            output='screen',
            parameters=[{
                'wheelbase': 0.20,
                'max_speed': 0.40,
                'max_steer': 0.60,
                'horizon_n': 10,
                'dt': 0.1
            }]
        ),

        # 8. Mostek sprzętowy I2C do ESP32 (silniki DC + serwo osi przedniej)
        Node(
            package='ov2slam',
            executable='esp32_i2c_bridge_node',
            name='esp32_i2c_bridge_node',
            output='screen',
            parameters=[{
                'i2c_device': '/dev/i2c-1',
                'i2c_address': 0x68,
                'wheel_radius': 0.03525,
                'ticks_per_rev': 660.0,
                'servo_center': 90,
                'servo_min': 40,
                'servo_max': 130,
                'servo_deg_per_rad': 45.0
            }]
        ),

        # 9. Połączenie drzewa TF (world -> odom dla Nav2)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_odom_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
        ),

        # 10. RViz2 (Wizualizacja)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])
