from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Path to your Gazebo world file
    world_file_path = '/home/oguzcan/otonom_ws/src/evata_sim/pist_world/arabasÄ±z.world'

    return LaunchDescription([
        # Launch Gazebo Classic with the specified world file
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file_path],
            output='screen'
        )
    ])
