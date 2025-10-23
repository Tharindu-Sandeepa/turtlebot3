import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define launch configurations for map file and simulation time
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch argument for the map file path
    declare_map_arg = DeclareLaunchArgument(
        'map_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('turtlebot3_maze'),
            'maps',
            'maze_assignment_map.yaml'
        ]),
        description='Full path to map yaml file to load'
    )
    
    # Declare launch argument for enabling simulation time
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Include Nav2 bringup launch file to initialize the navigation stack
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'map': map_file,  # Pass the map file to Nav2
            'use_sim_time': use_sim_time,  # Enable simulation time for Gazebo
            'params_file': PathJoinSubstitution([
                get_package_share_directory('turtlebot3_maze'),
                'config',
                'nav2_params.yaml'
            ])  # Load Nav2 configuration parameters
        }.items()
    )
    
    # Define the path to the RViz2 configuration file for navigation visualization
    rviz_config = PathJoinSubstitution([
        get_package_share_directory('turtlebot3_maze'),
        'rviz',
        'navigation.rviz'
    ])
    
    # Launch RViz2 node for visualizing the map and navigation
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],  # Load the specified RViz configuration
        parameters=[{'use_sim_time': use_sim_time}],  # Sync with simulation time
        output='screen'  # Display RViz2 output in the terminal
    )
    
    # Return the launch description with all defined arguments and nodes
    return LaunchDescription([
        declare_map_arg,
        declare_use_sim_time_arg,
        nav2_bringup,
        rviz_node
    ])
