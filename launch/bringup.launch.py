import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ov2slam')

    # Ścieżka do konfiguracji EKF
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    
    # Ścieżka do parametrów OV2SLAM (Euroc Mono)
    ov2slam_config_path = os.path.join(pkg_share, 'parameters_files', 'accurate', 'euroc', 'euroc_mono.yaml')
    
    # Ścieżka do konfiguracji RViz
    rviz_config_path = os.path.join(pkg_share, 'ov2slam_visualization.rviz') 

    planner_config_path = os.path.join(pkg_share, 'config', 'planner_server.yaml')
    return LaunchDescription([
        # 1. Węzeł IMU (MPU6050)
        Node(
            package='ov2slam',
            executable='mpu6050_node',
            name='mpu6050_node',
            output='screen'
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_config_path]
        ),

        # Menedżer cyklu życia dla węzłów Nav2 (aktywuje planner_server)
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
        # wezel ogarniania path ekf_filter_node
        Node(
            package='ov2slam',
            executable='path_planner_node',
            name='path_planner',
            output='screen',
        ),
        # 2. Główny węzeł OV2SLAM
        Node(
            package='ov2slam',
            executable='ov2slam_node',
            name='ov2slam_node',
            output='screen',
            arguments=[ov2slam_config_path]
        ),

        # 3. Węzeł dodający kowariancję do pozy VO
        Node(
            package='ov2slam',
            executable='pose_cov_adder_node',
            name='pose_cov_adder_node',
            output='screen'
        ),

        # 3b. Węzeł skali z enkoderów (nowy)
        Node(
            package='ov2slam',
            executable='skala_node',
            name='skala_node',
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

        # 6. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),

        # 7. Polaczenie drzewa TF (world -> odom dla Nav2)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_odom_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
        )
    ])
