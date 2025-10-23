from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Create and return a LaunchDescription with a list of launch actions
    return LaunchDescription([
        # Execute Gazebo simulator (gz sim) with the following parameters:
        # -r: run simulation
        # -v 3: verbose level 3
        # Load the maze_simple.world file
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-v', '3', 'worlds/maze_simple.world'],
            output='screen'  # Display output in the terminal
        ),
    ])