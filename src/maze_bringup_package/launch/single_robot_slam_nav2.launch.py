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

    # SLAM Toolbox
    slam_share = get_package_share_directory("slam_toolbox")
    slam_params = os.path.join(slam_share, "config", "mapper_params_online_async.yaml")
    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": use_sim_time}],
        remappings=[("scan", "/scan")],
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
        }.items(),
    )

    # Lifecycle manager for slam_toolbox 
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["slam_toolbox"],
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        gazebo_launch,
        slam_node,
        nav2_launch,
        lifecycle_manager_node,
    ])
