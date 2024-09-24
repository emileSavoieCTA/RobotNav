import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('robot_sim')

    # Get the URDF file path
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Make sure the URDF file exists
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")
    
    # Launch Gazebo Sim (Ignition Gazebo)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_desc, value_type=str)}]
    )

    # Spawn the robot in Gazebo Sim with a delay to ensure robot_state_publisher is running
    spawn_entity = TimerAction(
        period=3.0,  # Wait 3 seconds before spawning the robot
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-topic', '/robot_description',
                           '-name', 'robot',
                           '-allow_renaming', 'true'],
                output='screen'
            )
        ]
    )

    # Bridge to transfer data between ROS 2 and Gazebo Sim
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # '/world/empty/model/robot/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model',
            # '/model/robot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            # '/world/empty/model/robot/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            'robot/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'
        ],
        output='screen'
    )

    # Return the launch description
    return LaunchDescription([
        # gazebo,
        # robot_state_publisher,
        # spawn_entity,  # Delayed action
        bridge
    ])