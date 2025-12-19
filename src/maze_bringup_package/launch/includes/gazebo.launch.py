from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("maze_bringup_package")
    tb3_share = get_package_share_directory("turtlebot3_gazebo")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")

    maze_world = os.path.join(pkg_share, "worlds", world)

    set_tb3_models = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(tb3_share, "models"),
    )

    set_maze_models = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(pkg_share, "models"),
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -s -v2 ", maze_world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-g -v2 ",
            "on_exit_shutdown": "true",
        }.items(),
    )

    return LaunchDescription([
        set_tb3_models,
        set_maze_models,
        gzserver,
        gzclient,
    ])
