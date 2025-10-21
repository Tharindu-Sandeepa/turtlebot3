import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Fix for snap library conflicts
    ld_preload_unset = SetEnvironmentVariable(
        name='LD_PRELOAD',
        value=''
    )
    
    # Launch configuration
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_world_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('turtlebot3_maze'),
            'worlds',
            'maze_assignment.world'
        ]),
        description='Gazebo world file to load'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Gazebo server (headless - no GUI)
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_file],
        output='screen',
        additional_env={'LD_LIBRARY_PATH': '/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu'}
    )
    
    return LaunchDescription([
        ld_preload_unset,
        declare_world_arg,
        declare_use_sim_time_arg,
        gazebo_server,
    ])
