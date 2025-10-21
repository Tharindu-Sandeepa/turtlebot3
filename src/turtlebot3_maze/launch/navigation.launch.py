import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch configuration
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_map_arg = DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('turtlebot3_maze'),
            'maps',
            'maze_assignment_map.yaml'
        ]),
        description='Full path to map yaml file to load'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Nav2 Bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([
                get_package_share_directory('turtlebot3_maze'),
                'config',
                'nav2_params.yaml'
            ])
        }.items()
    )
    
    # RViz2 for navigation
    rviz_config = PathJoinSubstitution([
        get_package_share_directory('turtlebot3_maze'),
        'rviz',
        'navigation.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        declare_map_arg,
        declare_use_sim_time_arg,
        nav2_bringup,
        rviz_node
    ])
