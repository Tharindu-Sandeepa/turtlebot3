from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-v', '3', 'worlds/maze_simple.world'],
            output='screen'
        ),
    ])