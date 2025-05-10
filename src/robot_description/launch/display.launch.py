import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package paths
    robot_description_pkg = get_package_share_directory('robot_description')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    # File paths
    default_model_path = os.path.join(robot_description_pkg, 'src', 'description', 'robot.urdf.xacro')
    default_rviz_config_path = os.path.join(robot_description_pkg, 'rviz', 'config.rviz')
    nav2_launch_file = os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
    nav2_params_file = os.path.join(robot_description_pkg, 'config', 'agv_navigation_params.yaml')
    slam_params_file = os.path.join(robot_description_pkg, 'config', 'mapper_params_online_async.yaml')
    world_path = os.path.join(robot_description_pkg, 'world', 'catalyst_env.world')

    # Launch configuration variables
    gui = LaunchConfiguration('gui')
    model = LaunchConfiguration('model')
    rvizconfig = LaunchConfiguration('rvizconfig')

    # Process xacro
    doc = xacro.parse(open(default_model_path))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    use_sim_time = {'use_sim_time': True}

    # Nodes
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path]), **use_sim_time}],
        condition=UnlessCondition(gui)
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[use_sim_time],
        condition=IfCondition(gui)
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, use_sim_time]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'AGV5',
                   '-x', '0.0', '-y', '0.0', '-z', '0.0',
                   '-R', '0.0', '-P', '0', '-Y', '0.0'],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizconfig],
        parameters=[use_sim_time]
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/ray/lidar_points'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'lidar_front',
            'transform_tolerance': 0.1,
            'min_height': -0.1,
            'max_height': 0.1,
            'angle_min': -3.14,
            'angle_max': 3.14,
            'angle_increment': 0.01,
            'range_min': 0.2,
            'range_max': 100.0,
            'use_inf': False,
            'inf_epsilon': 1.0,
            **use_sim_time
        }],
        output='screen'
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, use_sim_time]
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file
        }.items()
    )

    keyboard_teleop = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='True', description='Enable joint_state_publisher_gui'),
        DeclareLaunchArgument('model', default_value=default_model_path, description='Robot model path'),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path, description='RViz config file'),

        # Simulation setup
        gazebo,
        joint_state_publisher,
        joint_state_publisher_gui,
        robot_state_publisher,
        spawn_entity,

        # Tools
        rviz,
        slam_toolbox,
        nav2,
        keyboard_teleop,
        pointcloud_to_laserscan_node
    ])