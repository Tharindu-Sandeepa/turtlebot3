#!/usr/bin/env python3
"""
Generate Assignment-Compliant Maze
Creates the specific maze layout required for IE4060 Assignment 02
"""

from maze_generator import MazeGenerator
import os

def create_assignment_maze():
    """Create the assignment-compliant maze with 0.5m passages"""
    
    # Assignment-specific maze layout (following Figure 1 specifications)
    assignment_layout = [
        "####################",
        "#        #         #",
        "#  ####  #  ####  ##",
        "#  #        #      #", 
        "#  #  #####  #### #",
        "#  #     #       #",
        "#  ##### #  #######",
        "#        #        #",
        "####  ####  ##### #",
        "#           #     #",
        "#  #####  # #  ####",
        "#  #      # #     #",
        "#  #  ##########  #",
        "#  #              #",
        "####################"
    ]
    
    generator = MazeGenerator(width=15, height=15, passage_width=0.5)
    
    # Create world content
    world_content = generator._layout_to_gazebo_world(assignment_layout)
    
    # Save to worlds directory
    worlds_dir = os.path.join(os.path.dirname(__file__), 'worlds')
    os.makedirs(worlds_dir, exist_ok=True)
    
    filepath = os.path.join(worlds_dir, 'maze_assignment.world')
    with open(filepath, 'w') as f:
        f.write(world_content)
    
    print(f"✅ Assignment-compliant maze created: {filepath}")
    print("   - Passage width: 0.5m (as required)")
    print("   - Wall height: 0.5m")
    print("   - TurtleBot3 start position: (1.0, 1.0)")
    
    return filepath

def create_test_mazes():
    """Create additional test mazes for comprehensive testing"""
    generator = MazeGenerator()
    
    # Simple maze for basic testing
    simple_maze = generator.generate_random_maze(complexity=0.3, density=0.3)
    generator.save_world_file(simple_maze, "maze_simple.world")
    print("✅ Simple test maze created: maze_simple.world")
    
    # Complex maze for challenge testing
    complex_maze = generator.generate_random_maze(complexity=0.8, density=0.7)
    generator.save_world_file(complex_maze, "maze_complex.world")
    print("✅ Complex test maze created: maze_complex.world")
    
    # Empty space for SLAM testing
    empty_world = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="empty_maze">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <model name="turtlebot3_burger">
      <include>
        <uri>model://turtlebot3_burger</uri>
        <pose>1.0 1.0 0.1 0 0 0</pose>
      </include>
    </model>
  </world>
</sdf>"""
    
    generator.save_world_file(empty_world, "empty.world")
    print("✅ Empty world created: empty.world")

def main():
    print("🎯 Generating IE4060 Assignment 02 Maze Environment")
    print("=" * 50)
    
    # Create assignment maze
    assignment_file = create_assignment_maze()
    
    # Create additional test mazes
    create_test_mazes()
    
    print("\n" + "=" * 50)
    print("✅ All maze worlds generated successfully!")
    print("\nNext steps:")
    print("1. Source your workspace: source install/setup.bash")
    print("2. Set TurtleBot3 model: export TURTLEBOT3_MODEL=burger")
    print("3. Launch SLAM: ros2 launch turtlebot3_maze slam.launch.py world:=maze_assignment.world")
    print("4. Build your map and save it")
    print("5. Test navigation: ros2 launch turtlebot3_maze navigation.launch.py")

if __name__ == "__main__":
    main()
