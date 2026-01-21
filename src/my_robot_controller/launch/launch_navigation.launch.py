import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'my_robot_controller'
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map', default=os.path.expanduser('~/hospital_map.yaml'))
    autostart = LaunchConfiguration('autostart', default='true')

    # Robot description
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare(pkg_name),
            'description',
            'robot.urdf.xacro'
        ])
    ])

    # Gazebo avec monde hôpital
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
            'verbose': 'false'
        }.items()
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'hospital_robot', 
                   '-x', '0', '-y', '0', '-z', '0.1'],
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

    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file
        }]
    )

    # AMCL (Localisation)
    nav2_params_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'config',
        'nav2_params.yaml'
    ])

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file]
    )

    # Controller Server avec REMAPPING pour cmd_vel
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('/cmd_vel', '/diff_cont/cmd_vel_unstamped')
        ]
    )

    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file]
    )

    # Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file]
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file]
    )

    # Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file]
    )

    # Lifecycle Manager pour Localisation (Map Server + AMCL)
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # Lifecycle Manager pour Navigation
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower'
            ]
        }]
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'rviz',
        'nav2_view.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ========== INTERFACE WEB NAV2 ==========
    # Interface Web de Contrôle et Navigation
    web_interface_node = Node(
        package='my_robot_controller',
        executable='nav2_hospital_interface.py',
        name='nav2_web_interface',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation time'),
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/hospital_map.yaml'),
                            description='Full path to map yaml file'),
        DeclareLaunchArgument('autostart', default_value='true',
                            description='Automatically startup the nav2 stack'),
        
        # Gazebo et Robot
        gazebo,
        robot_state_publisher,
        spawn_entity,
        spawn_diff_drive,
        spawn_joint_broad,
        
        # Localisation
        map_server,
        amcl,
        lifecycle_manager_localization,
        
        # Navigation avec Timer pour attendre que tout soit prêt
        TimerAction(
            period=3.0,
            actions=[
                controller_server,
                planner_server,
                behavior_server,
                bt_navigator,
                waypoint_follower,
                lifecycle_manager_navigation
            ]
        ),
        
        # RViz
        rviz_node,
        
        # Interface Web (lancée après un délai pour que Nav2 soit prêt)
        TimerAction(
            period=5.0,
            actions=[web_interface_node]
        )
    ])