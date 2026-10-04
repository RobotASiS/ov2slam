import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ov2slam')

    planner_config_path = os.path.join(pkg_share, 'config', 'planner_server.yaml')
    mpc_config_path = os.path.join(pkg_share, 'parameters_files', 'mpc_params.yaml')

    return LaunchDescription([
        # 1. Połączenie drzewa TF (world -> odom)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_odom_broadcaster',
            arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'odom']
        ),

        # 2. Węzeł symulatora podwozia (zastępuje fizycznego robota/kamery)
        Node(
            package='ov2slam',
            executable='sim_robot.py',
            name='sim_robot',
            output='screen'
        ),

        # 3. Serwer planowania Nav2
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_config_path]
        ),

        # 4. Menedżer cyklu życia dla Nav2
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

        # 5. Klient planisty ścieżki (odbiera kliknięcie /goal z GUI i pyta Nav2)
        Node(
            package='ov2slam',
            executable='path_planner_node',
            name='path_planner',
            output='screen',
        ),

        # 6. Interfejs GUI ImGui z mapą 2D (kliknięcie celu i wizualizacja pozycji)
        Node(
            package='ov2slam',
            executable='map2d_node',
            name='map2d_node',
            output='screen'
        ),

        # 7. Regulator MPC
        Node(
            package='ov2slam',
            executable='MPC.py',
            name='mpc_node',
            output='screen',
            parameters=[mpc_config_path]
        ),
    ])
