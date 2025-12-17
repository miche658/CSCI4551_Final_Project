# Multi-Robot Maze Navigation with Nav2

## Overview

This project implements a cooperative multi-robot navigation system using ROS 2 and the Nav2 navigation stack.
Two TurtleBot-style robots autonomously explore and solve a maze in the Gazebo simulation environment.
The robots build a shared map using SLAM and coordinate their exploration to avoid redundant work.

## System Architecture

- ROS 2 Humble

- Gazebo simulation

- SLAM Toolbox for mapping

- Nav2 for global A* path planning and local obstacle avoidance

- Custom coordination node for multi-robot task allocation

## Repository Structure

```
src/
├── maze_world/ # Gazebo maze world files
├── robot_description/ # URDF and sensor configuration
├── multirobot_bringup/ # Launch files for multi-robot Nav2
├── coordination_node/ # Custom ROS 2 node for task assignment
```

## Requirements

- Ubuntu 22.04

- ROS 2 Humble

- Gazebo (compatible with Humble)

- Nav2

- SLAM Toolbox

## Build Instructions

```
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Running the Project
Launch Single Robot (Testing)

```
ros2 launch multirobot_bringup single_robot.launch.py
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

- Nav2 global planning using built in A*

- Local obstacle avoidance with costmaps

- Dynamic obstacle handling in Gazebo

- Medium level multi-robot coordination

## Group 11 Team Members

- Joe Thomas 

- Martin Michelli

- Daniel Vu, Computer Science

## Future Work

- True multi-robot SLAM with map merging

- Advanced frontier based task allocation

- Learning based navigation improvements