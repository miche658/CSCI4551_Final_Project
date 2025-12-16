#!/usr/bin/env python3

'''
CSCI 4551 HW4 Launch File
'''

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from pathlib import Path

def generate_launch_description():
    # Get package directory:
    pkg_path = get_package_share_path('visual_ackermann_package')
    # Build path to the video file:
    video_path = str(Path(pkg_path) / 'videos' / 'bluedot.mp4')

    return LaunchDescription([
        # Turtlesim
        Node(package='turtlesim', executable='turtlesim_node', name='turtlesim'),
        
        # Video Publisher
        Node(package='visual_ackermann_package', executable='video_publisher_node', name='video_publisher',
        parameters=[{'video_path': video_path}]),
       
        # Perception Node
        Node(package='visual_ackermann_package', executable='perception_node', name='perception_node',
        parameters=[{'image_topic': '/camera/image_raw',
        'hsv_lower': [100, 150, 50], # Currently tuned to track blue objects
        'hsv_upper': [130, 255, 255]
        }]),
       
        # Control Node
        Node(package='visual_ackermann_package', executable='control_node', name='control_node')
    ])

