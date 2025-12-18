#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool, Hyungyu Kim
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, TimerAction, DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_share = get_package_share_directory('turtlebot3_gazebo')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Robot positions
    robot1_x = '-6.0'
    robot1_y = '-4.5'
    robot2_x = '-6.0'
    robot2_y = '-3.0'
    
    world = os.path.join(get_package_share_directory("maze_bringup_package"), "worlds", "maze_harmonic.sdf")
    
    # URDF path
    urdf_path = os.path.join(pkg_share, 'models', 'turtlebot3_waffle', 'model.sdf')
    
    # Start Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )
    
    # Start Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 ', 'on_exit_shutdown': 'true'}.items()
    )
    
    # Spawn Robot 1 using ros_gz_sim spawn command
    spawn_robot1 = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-name', 'robot1',
             '-x', robot1_x,
             '-y', robot1_y,
             '-z', '0.01',
             '-topic', '/robot1/robot_description'],
        output='screen'
    )
    
    # Robot 1 state publisher
    robot1_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'namespace': 'robot1'
        }.items()
    )
    
    # Spawn Robot 2 with delay
    spawn_robot2 = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-name', 'robot2',
             '-x', robot2_x,
             '-y', robot2_y,
             '-z', '0.01',
             '-topic', '/robot2/robot_description'],
        output='screen'
    )
    
    # Robot 2 state publisher
    robot2_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'namespace': 'robot2'
        }.items()
    )
    
    # Bridge for robot1
    bridge_robot1 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace='robot1',
        arguments=[
            '/robot1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/robot1/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/robot1/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/robot1/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen'
    )
    
    # Bridge for robot2
    bridge_robot2 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace='robot2',
        arguments=[
            '/robot2/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/robot2/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/robot2/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/robot2/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen'
    )
    
    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_share, 'models'))
    
    # Delay robot 2 by 5 seconds
    robot2_delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_robot2, robot2_state_publisher, bridge_robot2]
    )
    
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot1_state_publisher)
    ld.add_action(spawn_robot1)
    ld.add_action(bridge_robot1)
    ld.add_action(robot2_delayed_spawn)
    
    return ld
