# Multi-Robot Maze Navigation with Nav2

## Overview

This project implements a cooperative multi-robot navigation system using ROS 2.
Two TurtleBot-style robots autonomously explore and solve a maze in the Gazebo simulation environment.
The robots build a shared map using SLAM and coordinate their exploration to avoid redundant work.

## System Architecture

- ROS 2 Jazzy
- Gazebo Sim (gz-sim) + ros_gz_bridge
- turtlebot3 packages (model + description)
- SLAM Toolbox for mapping
- Custom coordination node for multi-robot task allocation

## Repository Structure

```
src/
├── maze_world/ # Gazebo maze world files
├── robot_description/ # URDF and sensor configuration
├── multirobot_bringup/ # Launch files for multi-robot Nav2
├── coordination_node/ # Custom ROS 2 node for task assignment
├── 
```

## Requirements
- Ubuntu 22.04
- ROS 2 Jazzy
- Gazebo
- SLAM Toolbox

## Workspace build
From the workspace root:

```bash
cd CSCI4551_Final_Project
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Build Instructions

Run (single robot + SLAM + frontier explorer)

Launch the maze world + robot + bridges + slam_toolbox + RViz + explorer:

```
ros2 launch maze_bringup_package launch_world.py
```

Launch Two Robots

```
ros2 launch multirobot_bringup multi_robot.launch.py
```

Run Coordination Node

```
ros2 run coordination_node coordinator
```

## Features

- Autonomous SLAM based mapping

- Custom Navigation with A*

- Local obstacle avoidance

- Multi-robot coordination

## Group 11 Team Members

- Joe Thomas 

- Martin Michelli

- Daniel Vu

## Future Work

