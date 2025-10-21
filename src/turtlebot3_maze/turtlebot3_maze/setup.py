from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'turtlebot3_maze'

setup(
    name=package_name,
    version='1.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pth')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@university.com',
    description='TurtleBot3 Maze Environment for IE4060 Assignment 02',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'maze_gui = turtlebot3_maze.maze_gui:main',
            'customize_maze = turtlebot3_maze.customize_maze:main',
            'generate_assignment_maze = turtlebot3_maze.generate_assignment_maze:main',
            'dqn_navigation = turtlebot3_maze.dqn_navigation:main',
            'performance_tester = turtlebot3_maze.performance_tester:main',
        ],
    },
)
