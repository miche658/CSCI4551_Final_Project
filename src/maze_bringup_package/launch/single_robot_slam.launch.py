from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )

    # Gazebo + TB3
    tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_share, "launch", "turtlebot3_world.launch.py")
        )
    )

    # SLAM Toolbox (async)
    slam_share = get_package_share_directory("slam_toolbox")
    slam_params = os.path.join(slam_share, "config", "mapper_params_online_async.yaml")

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": use_sim_time}],
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        gazebo_launch,
        slam_node,
    ])
