from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    pkg_share = get_package_share_directory("maze_bringup_package")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")

    declare_map = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(pkg_share, "maps", "maze_map.yaml"),
    )

    declare_params = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_share, "maps", "params_file.yaml"),
    )

    tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_share, "launch", "turtlebot3_world.launch.py")
        )
    )

    nav2_share = get_package_share_directory("nav2_bringup")
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam": "True",
            "map": map_yaml,
            "params_file": params_file,
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params,
        gazebo_launch,
        nav2_launch,
    ])
