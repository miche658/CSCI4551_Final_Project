#!/usr/bin/env python3

'''
CSCI 4551 Final Project Gazebo Launch File
Launches Gazebo, Turtlebot3 Waffle, the Maze world, and the solver node
Make sure .bashrc has the following line:
export TURTLEBOT3_MODEL=waffle
'''

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory:
    gazebo_dir = get_package_share_directory("turtlebot3_gazebo")

    maze_bringup_pkg = FindPackageShare('maze_bringup_package').find('maze_bringup_package')
    
    # Use simulation time:
    sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    world_file = os.path.join(maze_bringup_pkg, 'worlds', 'maze.world')
    
    # Launch Gazebo and Turtlebot:
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_dir, 'launch', 'turtlebot3_world.launch.py')), launch_arguments={'world': world_file}.items())

    # Launch the Maze Solver node:
    maze_solver_node = Node(
        package='maze_solver_package',
        executable='maze_solver_node',
        name='maze_solver',
        output='screen',
        parameters=[{'use_sim_time': sim_time}]
    )
    
    return LaunchDescription([
        # Gazebo:
        gazebo_launch,
        # Maze Solver Node:
        maze_solver_node,
    ])

