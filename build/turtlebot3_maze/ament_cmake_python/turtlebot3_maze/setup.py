from setuptools import find_packages
from setuptools import setup

setup(
    name='turtlebot3_maze',
    version='1.2.0',
    packages=find_packages(
        include=('turtlebot3_maze', 'turtlebot3_maze.*')),
)
