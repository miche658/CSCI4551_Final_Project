from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
    SetEnvironmentVariable,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    # ----------------------------
    # Paths
    # ----------------------------
    pkg_share = get_package_share_directory("maze_bringup_package")
    tb3_share = get_package_share_directory("turtlebot3_gazebo")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    # Hardcoded world (NO terminal override needed)
    maze_world = os.path.join(pkg_share, "worlds", "maze_harmonic.sdf")

    # ----------------------------
    # Launch configs
    # ----------------------------
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    params_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(pkg_share, "config", "params.yaml"),
    )
    x_pose = LaunchConfiguration("x_pose", default="-6.0")
    y_pose = LaunchConfiguration("y_pose", default="-4.5")

    # ----------------------------
    # Environment
    # ----------------------------
    set_tb3_model = SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger")

    set_tb3_models = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(tb3_share, "models"),
    )

    set_maze_models = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(pkg_share, "models"),
    )

    # ----------------------------
    # Gazebo Sim
    # ----------------------------
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

    # ----------------------------
    # TurtleBot3 spawn + TF
    # ----------------------------
    tb3_launch_dir = os.path.join(tb3_share, "launch")

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_launch_dir, "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    # ----------------------------
    # Twist converter
    # ----------------------------
    twist_converter = Node(
        package="cmdvel_tools",
        executable="twist_to_stamped",
        output="screen",
        parameters=[
            {"in_topic": "/cmd_vel_nav"},
            {"out_topic": "/cmd_vel"},
            {"frame_id": "base_link"},
            {"use_sim_time": use_sim_time},
        ],
    )

    # ----------------------------
    # Nav2 (SLAM mode)
    # ----------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam": "True",
            "params_file": params_file,
        }.items(),
    )

    explorer = Node(
        package="maze_bringup_package",
        executable="frontier_explorer",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Delayed Nav2 and explorer start to ensure Gazebo is ready
    delayed_nav2_launch = TimerAction(
        period=12.0,
        actions=[nav2_launch, explorer],
    )
    

    # ----------------------------
    # EKF (optional)
    # ----------------------------
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config", "ekf.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        set_tb3_model,
        set_tb3_models,
        set_maze_models,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_tb3,
        ekf,
        twist_converter,
        delayed_nav2_launch,
    ])

