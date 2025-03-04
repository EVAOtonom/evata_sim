import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Mevcut çalışma dizinini al
    dir_path = os.path.dirname(os.path.realpath(__file__))

    # /install/evata_sim/share/evata_sim/launch kısmına kadar yolu al
    src_dir = dir_path.split('/install')[0]  # install kısmını çıkar

    # pist_world klasörüne ve pist.world dosyasına giden yolu oluştur
    world_file = os.path.join(src_dir, 'src', 'evata_sim', 'pist_world', 'arabasız_pist.world')

    # Models klasörüne giden yolu oluştur
    models_path = os.path.join(src_dir, 'src', 'evata_sim', 'pist_world', 'models')
    sdf = "/home/oguzcan/otonom_ws/src/evata_sim/pist_world/models/Evata/model.sdf"
    # Gazebo'nun environment variable'larını ayarla
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = models_path
    doc = xacro.parse(open(sdf))
    xacro.process_doc(doc)
    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file],
        output='screen'
    )
    # -----------------------------------------------------------------------------------
    map_dir = LaunchConfiguration(
        'map',
        default="/home/oguzcan/otonom_ws/src/evata_sim/map/my_map.yaml"       
    )
    param_file_name = "waffle.yaml"
    param_dir = LaunchConfiguration(
        'params_file',
        default="/home/oguzcan/otonom_ws/src/evata_sim/params/waffle.yaml")
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')
    # -----------------------------------------------------------------------------------------
    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=['-entity', "Evata",
                   '-name', "Evata",
                   '-file', "/home/oguzcan/otonom_ws/src/evata_sim/pist_world/models/Evata/model.sdf",
                   '-allow_renaming', 'true',
                   '-x', '-2.0',
                   '-y', '-0.5',
                   '-z', '0.01'],
        )
    # Köprü oluşturacak node
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
		'/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
		'/model/Evata/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
		'/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
		'/camera/rgb@sensor_msgs/msg/Image@ignition.msgs.Image',
		'/lidar/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
		'/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
        '/odom/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        '/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
        '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
		'/depth_camera/zed/image@sensor_msgs/msg/Image@ignition.msgs.Image',
		'/depth_camera/zed/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloud',
		'/depth_camera/zed/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
		'/depth_camera/zed/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
		'/depth_camera/zed/image@sensor_msgs/msg/Image@ignition.msgs.Image',
		'/depth_camera/zed/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked'


            
            
        ],
        remappings=[
            ("/odom/tf", "tf"),
        ],
        output='screen'
    )

    map_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        # -------------------------------------------------------------
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),
        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),
        #---------------------------------------------------------------
        gazebo,
        bridge,
        map_static_tf,
        ignition_spawn_entity,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': doc.toxml()}]),
        #---------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
            
            Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_imu',
    arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link'],
    output='screen'
    ),
    Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_scan',
    arguments=['0', '0', '0.1', '0', '0', '0', 'odom', 'base_link'],
    output='screen'
    ),
    Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_footprint_to_base_link',
    arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'wheel_left_link'],
    output='screen'
    ),
    Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_footprint_to_base_link',
    arguments=['2', '2', '4', '0', '0', '0', 'map', 'odom'],
    output='screen'
    ),
    Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_footprint_to_base_link',
    arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'wheel_right_link'],
    output='screen'
    ),
    Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_footprint_to_base_link',
    arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'caster_back_right_link'],
    output='screen'
    ),
    Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_footprint_to_base_link',
    arguments=['0', '0', '0.1', '0', '0', '0', 'lidar_link', '/scan'],
    output='screen'
    ),
    Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_footprint_to_base_link',
    arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'lidar_link'],
    output='screen'
    ),
    Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_footprint_to_base_link',
    arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'caster_back_left_link'],
    output='screen'
    ),
    Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_footprint_to_base_link',
    arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link'],
    output='screen'
    ),
    Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_footprint_to_base_link',
    arguments=['0', '0', '0.1', '0', '0', '0', 'camera_link', 'camera_rgb_frame'],
    output='screen'
    )

    ])
        #-------------------------------------------------------------

