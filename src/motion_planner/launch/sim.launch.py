from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Set TurtleBot3 model type
    env_var = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi')

    # Path to TurtleBot3 Waffle Pi SDF model
    sdf_model_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        'turtlebot3_waffle_pi',
        'model.sdf'
    )

    # Set GAZEBO_MODEL_PATH
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(get_package_share_directory('my_gazebo_worlds'), 'models')
    )

    
    # Paths
    world_path = os.path.join(
        get_package_share_directory('my_gazebo_worlds'),
        'worlds', 'world.sdf'
    )

    # Launch Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={
            'world': world_path,
            'verbose': 'false',
            'extra_gazebo_args': '--ros-args --params-file /opt/ros/humble/share/gazebo_ros/launch/gazebo_ros_factory.yaml'
        }.items()
    )

    # Spawn TurtleBot3 with 5-second delay
    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'waffle_pi',
                    '-file', sdf_model_path,
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.01',
                    '-Y', '0.0'
                ],
                output='screen'
            )
        ]
    )

    # SLAM node (Simultaneous Localization and Mapping) after robot is spawned
    slam = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': True, 'slam_toolbox': True, 'queue_size': 100}],
    )

    # RViz visualization after robot is spawned and SLAM starts
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        env_var,
        gazebo_model_path,
        gazebo,
        spawn_entity,
        Node(
            package='motion_planner',
            executable='motion_planner_node',
            output='screen'
        ),

        rviz,
        slam

    ])
