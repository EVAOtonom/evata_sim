import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Mevcut çalışma dizinini al
    dir_path = os.path.dirname(os.path.realpath(__file__))

    # /install/evata_sim/share/evata_sim/launch kısmına kadar yolu al
    src_dir = dir_path.split('/install')[0]  # install kısmını çıkar

    # pist_world klasörüne ve pist.world dosyasına giden yolu oluştur
    world_file = os.path.join(src_dir, 'src', 'evata_sim', 'pist_world', 'pist.world')

    # Models klasörüne giden yolu oluştur
    models_path = os.path.join(src_dir, 'src', 'evata_sim', 'pist_world', 'models')

    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo
    ])

