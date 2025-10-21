#!/usr/bin/env python3
"""
Core maze generator for TurtleBot3 assignment
Generates Gazebo-compatible world files with proper 0.5m passages
"""

import numpy as np
import math
import os
from pathlib import Path

class MazeGenerator:
    def __init__(self, width=10, height=10, passage_width=0.5):
        self.width = width
        self.height = height
        self.passage_width = passage_width  # 0.5m as per assignment requirements
        self.wall_height = 0.5
        self.wall_thickness = 0.1
        self.wall_color = "Gazebo/Red"
        
    def generate_assignment_maze(self):
        """Generate the assignment-compliant maze from Figure 1"""
        # Create a simple maze layout that meets 0.5m passage requirement
        maze_layout = [
            "####################",
            "#                  #",
            "#  ####  ####  ####",
            "#  #           #  #",
            "#  #  #####  # #  #",
            "#  #     #    #   #",
            "#  ##### #  ####  #",
            "#        #        #",
            "####  ####  ####  #",
            "#           #     #",
            "#  #####  # #  ####",
            "#  #      # #     #",
            "#  #  ##########  #",
            "#  #              #",
            "####################"
        ]
        
        return self._layout_to_gazebo_world(maze_layout)
    
    def generate_random_maze(self, complexity=0.75, density=0.75):
        """Generate a random maze using depth-first search"""
        # Only shape can be 0 or 1
        shape = ((self.height // 2) * 2 + 1, (self.width // 2) * 2 + 1)
        
        # Adjust complexity and density relative to maze size
        complexity = int(complexity * (5 * (shape[0] + shape[1])))
        density = int(density * ((shape[0] // 2) * (shape[1] // 2)))
        
        # Build actual maze
        Z = np.zeros(shape, dtype=bool)
        
        # Fill borders
        Z[0, :] = Z[-1, :] = 1
        Z[:, 0] = Z[:, -1] = 1
        
        # Make aisles
        for i in range(density):
            x, y = np.random.randint(0, shape[1]//2 + 1) * 2, np.random.randint(0, shape[0]//2 + 1) * 2
            Z[y, x] = 1
            for j in range(complexity):
                neighbours = []
                if x > 1:             neighbours.append((y, x - 2))
                if x < shape[1] - 2:  neighbours.append((y, x + 2))
                if y > 1:             neighbours.append((y - 2, x))
                if y < shape[0] - 2:  neighbours.append((y + 2, x))
                if len(neighbours):
                    y_, x_ = neighbours[np.random.randint(0, len(neighbours))]
                    if Z[y_, x_] == 0:
                        Z[y_, x_] = 1
                        Z[y_ + (y - y_) // 2, x_ + (x - x_) // 2] = 1
                        x, y = x_, y_
        
        return self._array_to_gazebo_world(Z)
    
    def _layout_to_gazebo_world(self, layout):
        """Convert text layout to Gazebo world"""
        world_template = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="maze_assignment">
    <include>
      <uri>model://sun</uri>
    </include>
    
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Maze Walls -->
{walls}
    
    <!-- TurtleBot3 Start Position -->
    <model name="turtlebot3_burger">
      <include>
        <uri>model://turtlebot3_burger</uri>
        <pose>1.0 1.0 0.1 0 0 0</pose>
      </include>
    </model>
    
  </world>
</sdf>"""
        
        walls_xml = ""
        wall_id = 0
        
        for y, row in enumerate(layout):
            for x, cell in enumerate(row):
                if cell == '#':
                    pos_x = x * self.passage_width
                    pos_y = y * self.passage_width
                    
                    wall_xml = f"""
    <model name="maze_wall_{wall_id}">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{self.wall_thickness} {self.wall_thickness} {self.wall_height}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{self.wall_thickness} {self.wall_thickness} {self.wall_height}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>{self.wall_color}</name>
            </script>
          </material>
        </visual>
      </link>
      <pose>{pos_x} {pos_y} {self.wall_height/2} 0 0 0</pose>
    </model>"""
                    walls_xml += wall_xml
                    wall_id += 1
        
        return world_template.format(walls=walls_xml)
    
    def _array_to_gazebo_world(self, maze_array):
        """Convert numpy array to Gazebo world"""
        world_template = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="maze_random">
    <include>
      <uri>model://sun</uri>
    </include>
    
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Maze Walls -->
{walls}
    
    <!-- TurtleBot3 Start Position -->
    <model name="turtlebot3_burger">
      <include>
        <uri>model://turtlebot3_burger</uri>
        <pose>1.0 1.0 0.1 0 0 0</pose>
      </include>
    </model>
    
  </world>
</sdf>"""
        
        walls_xml = ""
        wall_id = 0
        
        for y in range(maze_array.shape[0]):
            for x in range(maze_array.shape[1]):
                if maze_array[y, x]:
                    pos_x = x * self.passage_width / 2
                    pos_y = y * self.passage_width / 2
                    
                    wall_xml = f"""
    <model name="maze_wall_{wall_id}">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{self.wall_thickness} {self.wall_thickness} {self.wall_height}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{self.wall_thickness} {self.wall_thickness} {self.wall_height}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>{self.wall_color}</name>
            </script>
          </material>
        </visual>
      </link>
      <pose>{pos_x} {pos_y} {self.wall_height/2} 0 0 0</pose>
    </model>"""
                    walls_xml += wall_xml
                    wall_id += 1
        
        return world_template.format(walls=walls_xml)
    
    def save_world_file(self, world_content, filename):
        """Save world content to file"""
        worlds_dir = Path(__file__).parent / "worlds"
        worlds_dir.mkdir(exist_ok=True)
        
        filepath = worlds_dir / filename
        with open(filepath, 'w') as f:
            f.write(world_content)
        
        print(f"World file saved: {filepath}")
        return filepath

def main():
    """Test maze generation"""
    generator = MazeGenerator()
    
    # Generate assignment maze
    assignment_maze = generator.generate_assignment_maze()
    generator.save_world_file(assignment_maze, "maze_assignment.world")
    
    # Generate random maze for testing
    random_maze = generator.generate_random_maze()
    generator.save_world_file(random_maze, "maze_random.world")
    
    print("Maze generation completed!")

if __name__ == "__main__":
    main()
