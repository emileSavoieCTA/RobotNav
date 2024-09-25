from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_sim')
    
    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Spawn the robot
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'velodyne_lidar'],
                        output='screen')

    # Load the URDF
    urdf = os.path.join(pkg_share, 'urdf', 'velodyne_lidar.urdf')
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # Start Octomap server
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[
            {'resolution': 0.05},
            {'frame_id': 'map'},
            {'sensor_model.max_range': 5.0},
        ],
        remappings=[('cloud_in', '/velodyne_points')],
    )

    # Start RViz
    rviz_config = os.path.join(pkg_share, 'config', 'lidar_sim.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # Start LiDAR processor node
    lidar_processor = Node(
        package='lidar_sim',
        executable='lidar_processor',
        name='lidar_processor',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        robot_state_publisher,
        octomap_server,
        rviz,
        lidar_processor
    ])