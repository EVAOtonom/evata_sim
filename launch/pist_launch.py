from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Path to the .world
    world_file = os.path.join(
        os.getenv('HOME'), 'ros2_ws', 'src', 'evata_sim', 'pist_world', 'pist.world'
    )

    # Path to the models directory
    models_path = os.path.join(
        os.getenv('HOME'), 'ros2_ws', 'src', 'evata_sim', 'pist_world','models'
    )

    # Set the GAZEBO_MODEL_PATH environment variable
    os.environ['GAZEBO_MODEL_PATH'] = models_path

    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file],
        output='screen'
    )

    return LaunchDescription([
        gazebo
    ])

