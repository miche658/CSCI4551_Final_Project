from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")

    pkg_share = get_package_share_directory("maze_bringup_package")
    default_params = os.path.join(pkg_share, "config", "params.yaml")
    declare_params_file = DeclareLaunchArgument("params_file", default_value=default_params)

    # Gazebo + TurtleBot3
    tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_share, "launch", "turtlebot3_world.launch.py")
        )
    )
    
    twist_converter = Node(
        package="cmdvel_tools",
        executable="twist_to_stamped",
        name="twist_to_stamped",
        output="screen",
        parameters=[
            {"in_topic": "/cmd_vel_nav"},
            {"out_topic": "/cmd_vel"},
            {"frame_id": "base_link"},
            {"use_sim_time": use_sim_time},
        ],
    )

    # Nav2 bringup (SLAM mode: uses /map from slam_toolbox)
    nav2_share = get_package_share_directory("nav2_bringup")
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam": "True",
            "params_file": params_file,
            "use_waypoint_follower": "True",
        }.items(),
    )
    
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config", "ekf.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        gazebo_launch,
        ekf,
        twist_converter,
        nav2_launch,
    ])
