#!/usr/bin/env python3

import os
import launch
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch.substitutions


def generate_launch_description():

    robot_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi')
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value='/home/dacossti/llm_ros2_humble/src/my_gazebo_worlds/models'
    )

    llm_nav_dir = get_package_share_directory('llm_nav')
    world_path = os.path.join(get_package_share_directory('my_gazebo_worlds'), 'worlds', 'world.sdf')
    model_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models', 'turtlebot3_waffle_pi', 'model.sdf')
    urdf_path = os.path.join(llm_nav_dir, 'urdf', 'turtlebot3_waffle_pi.urdf')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={'world': world_path}.items()
    )

    spawn_entity = TimerAction(
        period=5.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'waffle_pi',
                '-file', model_path,
                '-x', '0.0', '-y', '0.0', '-z', '0.01'
            ],
            output='screen'
        )]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': launch.substitutions.Command(['xacro ', urdf_path])
        }]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    teleop_node = Node(
        package='turtlebot3_teleop',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[{'use_sim_time': True}]
    )

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        robot_model,
        gazebo_model_path,
        gazebo_launch,
        spawn_entity,
        robot_state_publisher,
        slam_node,
        rviz2_node,
        teleop_node
    ])
