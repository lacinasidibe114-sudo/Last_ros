import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'my_robot_controller'
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot description
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare(pkg_name),
            'description',
            'robot.urdf.xacro'
        ])
    ])

    # Gazebo avec le monde de l'hôpital
    world_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'worlds',
        'hospital_world.world'
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'hospital_robot', '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    # Controllers
    spawn_diff_drive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output="screen"
    )

    spawn_joint_broad = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen"
    )

    # SLAM Toolbox
    slam_params_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'config',
        'mapper_params_online_async.yaml'
    ])

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'rviz',
        'slam_view.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Interface Web
    web_interface_node = Node(
        package='my_robot_controller',
        executable='web_control_interface.py',
        name='web_control_interface',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        spawn_diff_drive,
        spawn_joint_broad,
        slam_toolbox_node,
        rviz_node,
        web_interface_node
    ])