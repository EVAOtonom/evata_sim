import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Mevcut çalışma dizinini al
    dir_path = os.path.dirname(os.path.realpath(__file__))
    
    # Harita ve parametre dosyalarının dinamik yollarını al
    src_dir = dir_path.split('/install')[0]  
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(src_dir,"src","evata_sim","navigasyon", 'map', 'harita.yaml')
    )
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(src_dir,"src","evata_sim","navigasyon", 'params', 'evata.yaml')
    )

    # nav2_launch_file_dir ve rviz_config_dir yolları
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_dir = os.path.join(src_dir,"src","evata_sim","navigasyon","nav2_evata_view.rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # 1. SADECE NAVİGASYON DÜĞÜMLERİNİ BAŞLAT (AMCL ve MAP HARİÇ)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': param_dir
            }.items(),
        ),

        # 2. HARİTA SUNUCUSUNU (MAP SERVER) MANUEL BAŞLAT
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_dir}, {'use_sim_time': use_sim_time}]
        ),

        # 3. SADECE HARİTA SUNUCUSU İÇİN LIFECYCLE MANAGER (AMCL SİLİNDİ)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        ),

        # 4. RViz'i başlat
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
