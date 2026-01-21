import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, 
    DeclareLaunchArgument, 
    TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    🏥 HospiBot - Launch File Unifié SLAM + Nav2 (VERSION CORRIGÉE)
    
    Ce launch file supporte deux modes :
    - mode:=slam  → Cartographie avec SLAM Toolbox + RViz SLAM
    - mode:=nav2  → Navigation avec Nav2 + RViz Nav2
    
    🔥 CORRECTIONS AJOUTÉES :
    - Velocity Smoother pour lissage des commandes
    - Collision Monitor pour sécurité
    - Smoother Server pour trajectoires fluides
    
    Usage:
        # Mode SLAM (cartographie)
        ros2 launch my_robot_bringup hospibot_unified.launch.py mode:=slam
        
        # Mode Nav2 (navigation)
        ros2 launch my_robot_bringup hospibot_unified.launch.py mode:=nav2 map:=~/hospital_map.yaml
    """
    
    pkg_name = 'my_robot_controller'
    
    # ========== ARGUMENTS ==========
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='slam',
        description='Mode: slam (cartographie) ou nav2 (navigation)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Utiliser le temps de simulation'
    )
    
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('/home/lacina/Last_ros/src/my_robot_controller/maps/hospital_map.yaml'),
        description='Chemin vers la carte (requis en mode nav2)'
    )
    
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Démarrer automatiquement la stack Nav2'
    )
    
    # ========== CONFIGURATIONS ==========
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    
    # Conditions pour les modes
    is_slam_mode = PythonExpression(["'", mode, "' == 'slam'"])
    is_nav2_mode = PythonExpression(["'", mode, "' == 'nav2'"])
    
    # ========== ROBOT DESCRIPTION ==========
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare(pkg_name),
            'description',
            'robot.urdf.xacro'
        ])
    ])
    
    # ========== GAZEBO (COMMUN) ==========
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
    
    # ========== SPAWN ROBOT (COMMUN) ==========
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'hospital_robot',
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # ========== ROBOT STATE PUBLISHER (COMMUN) ==========
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    
    # ========== CONTROLLERS (COMMUN) ==========
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
    
    # ========================================================================
    # MODE SLAM - Cartographie
    # ========================================================================
    
    slam_params_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'config',
        'mapper_params_online_async.yaml'
    ])
    
    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(is_slam_mode)
    )
    
    # RViz pour SLAM
    rviz_slam_config = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'rviz',
        'slam_view.rviz'
    ])
    
    rviz_slam = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_slam_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(is_slam_mode)
    )
    
    # ========================================================================
    # MODE NAV2 - Navigation
    # ========================================================================
    
    nav2_params_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'config',
        'nav2_params.yaml'
    ])
    
    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file
        }],
        condition=IfCondition(is_nav2_mode)
    )
    
    # AMCL
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file],
        condition=IfCondition(is_nav2_mode)
    )
    
    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('/cmd_vel', '/diff_cont/cmd_vel_unstamped')
        ],
        condition=IfCondition(is_nav2_mode)
    )
    
    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file],
        condition=IfCondition(is_nav2_mode)
    )
    
    # 🔥 SMOOTHER SERVER (NOUVEAU)
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params_file],
        condition=IfCondition(is_nav2_mode)
    )
    
    # Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file],
        condition=IfCondition(is_nav2_mode)
    )
    
    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file],
        condition=IfCondition(is_nav2_mode)
    )
    
    # Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file],
        condition=IfCondition(is_nav2_mode)
    )
    
    # 🔥 VELOCITY SMOOTHER (NOUVEAU)
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ],
        condition=IfCondition(is_nav2_mode)
    )
    
    # 🔥 COLLISION MONITOR (NOUVEAU)
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[nav2_params_file],
        condition=IfCondition(is_nav2_mode)
    )
    
    # Lifecycle Manager - Localisation
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl']
        }],
        condition=IfCondition(is_nav2_mode)
    )
    
    # 🔥 Lifecycle Manager - Navigation (MODIFIÉ avec nouveaux nodes)
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
                'smoother_server',      # 🔥 AJOUTÉ
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'     # 🔥 AJOUTÉ
            ]
        }],
        condition=IfCondition(is_nav2_mode)
    )
    
    # RViz pour Nav2
    rviz_nav2_config = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'rviz',
        'nav2_view.rviz'
    ])
    
    rviz_nav2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_nav2_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(is_nav2_mode)
    )
    
    # ========== INTERFACE WEB HOSPIBOT (COMMUN) ==========
    hospibot_interface = Node(
        package='my_robot_controller',
        executable='unified_web_interface.py',
        name='hospibot_interface',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # ========================================================================
    # LAUNCH DESCRIPTION FINALE
    # ========================================================================
    return LaunchDescription([
        # ===== ARGUMENTS =====
        mode_arg,
        use_sim_time_arg,
        map_yaml_arg,
        autostart_arg,
        
        # ===== COMPOSANTS COMMUNS =====
        gazebo,
        robot_state_publisher,
        spawn_entity,
        
        # Controllers avec délai (attendre Gazebo)
        TimerAction(
            period=2.0,
            actions=[
                spawn_diff_drive,
                spawn_joint_broad
            ]
        ),
        
        # ===== MODE SLAM =====
        # Lance SLAM Toolbox + RViz SLAM si mode=slam
        TimerAction(
            period=3.0,
            actions=[slam_toolbox_node]
        ),
        TimerAction(
            period=4.0,
            actions=[rviz_slam]
        ),
        
        # ===== MODE NAV2 =====
        # Lance Map Server + AMCL + Lifecycle Manager si mode=nav2
        TimerAction(
            period=3.0,
            actions=[
                map_server,
                amcl,
                lifecycle_manager_localization
            ]
        ),
        
        # 🔥 Lance Navigation Stack + Nouveaux Composants + RViz Nav2
        TimerAction(
            period=5.0,
            actions=[
                controller_server,
                planner_server,
                smoother_server,        # 🔥 AJOUTÉ
                behavior_server,
                bt_navigator,
                waypoint_follower,
                velocity_smoother,      # 🔥 AJOUTÉ
                collision_monitor,      # 🔥 AJOUTÉ
                lifecycle_manager_navigation
            ]
        ),
        TimerAction(
            period=6.0,
            actions=[rviz_nav2]
        ),
        
        # ===== INTERFACE WEB =====
        # Lance HospiBot Interface (toujours)
        TimerAction(
            period=7.0,
            actions=[hospibot_interface]
        )
    ])