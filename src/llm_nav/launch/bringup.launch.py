#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, LogInfo, TimerAction
)
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    instruction_arg = DeclareLaunchArgument(
        'instruction',
        default_value='Go to the living room',
        description='High-level user instruction'
    )
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2',
        default_value='false',
        description='Whether to launch the full Nav2 stack (planner, controller, BT, etc.)'
    )
    use_detector_arg = DeclareLaunchArgument(
        'use_detector',
        default_value='false',
        description='Whether to run the room detector node'
    )

    # Environment setup
    robot_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi')
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(get_package_share_directory('my_gazebo_worlds'), 'models')
    )

    # Paths
    llm_nav_dir = get_package_share_directory('llm_nav')
    world_path = os.path.join(get_package_share_directory('my_gazebo_worlds'), 'worlds', 'small_house.world')
    model_path = '/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf'
    urdf_path = '/opt/ros/humble/share/turtlebot3_gazebo/urdf/turtlebot3_waffle_pi.urdf'
    nav2_params_path = os.path.join(llm_nav_dir, 'params', 'nav2_params.yaml')
    rviz_config = os.path.join(llm_nav_dir, 'rviz', 'llm_nav.rviz')

    # Launch gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={'world': world_path}.items()
    )

    # Robot state publisher
    robot_description = open(urdf_path).read()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description
        }]
    )

    # Spawn robot in gazebo
    spawn_entity = TimerAction(
        period=45.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'waffle_pi',
                    '-file', model_path,
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.01'
                ],
                output='screen'
            )
        ]
    )

    # Core nodes (always launched)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params_path]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_path]
    )

    nav_to_pose = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='llm_nav',
                executable='nav_to_pose',
                name='nav_to_pose',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'instruction': LaunchConfiguration('instruction'),
                    'use_nav2': LaunchConfiguration('use_nav2')
                }]
            )
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    detector = Node(
        package='llm_nav',
        executable='room_detector',
        name='room_detector',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_detector')),
        parameters=[{'use_sim_time': True}]
    )

    # Conditional Nav2 stack nodes (only if use_nav2:=true)
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_path],
        condition=IfCondition(LaunchConfiguration('use_nav2'))
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_path],
        condition=IfCondition(LaunchConfiguration('use_nav2'))
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_path],
        condition=IfCondition(LaunchConfiguration('use_nav2'))
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_path, {'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_nav2'))
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'bt_navigator',
                'behavior_server'
            ]
        }],
        condition=IfCondition(LaunchConfiguration('use_nav2'))
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_nav2')),
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    return LaunchDescription([
        instruction_arg,
        use_nav2_arg,
        use_detector_arg,
        robot_model,
        gazebo_model_path,
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
        map_server,
        amcl,
        rviz,
        nav_to_pose,
        detector,
        planner_server,
        controller_server,
        bt_navigator,
        behavior_server,
        lifecycle_manager_navigation,
        lifecycle_manager_localization,
        LogInfo(msg='Bringup successfully executed using static map!')
    ])
