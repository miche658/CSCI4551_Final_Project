from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
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
    
    # Get package directories
    tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    #tb3_description_share = get_package_share_directory("turtlebot3_description")
    maze_bringup_share = get_package_share_directory("maze_bringup_package")
    
    # Launch Gazebo Harmonic with world file from package
    world_file = os.path.join(maze_bringup_share, "worlds", "maze_harmonic.sdf")
    gazebo_launch = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    
    # Get model name from environment
    model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    
    # Get URDF file path
    urdf_file = os.path.join(tb3_gazebo_share, 'models', f'turtlebot3_{model}', 'model.sdf')
    
    # Read URDF
    with open(urdf_file, 'r') as file:
        robot_desc = file.read()
    
    # Robot 1 - robot_state_publisher
    robot_state_publisher_1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='tb3_0',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
            'frame_prefix': 'tb3_0/'
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # Robot 1 - spawn in Gazebo Harmonic
    spawn_robot1 = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 5 && ros2 run ros_gz_sim create -name tb3_0 -topic /tb3_0/robot_description -x -6.0 -y -4.5 -z 0.01'],
        output='screen'
    )
    
    # Robot 2 - robot_state_publisher
    robot_state_publisher_2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='tb3_1',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
            'frame_prefix': 'tb3_1/'
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # Robot 2 - spawn in Gazebo Harmonic with delay
    spawn_robot2 = ExecuteProcess(
        cmd=['bash', '-c', 'sleep 8 && ros2 run ros_gz_sim create -name tb3_1 -topic /tb3_1/robot_description -x -7.0 -y -4.5 -z 0.01'],
        output='screen'
    )
    
    # Bridge topics from Gazebo to ROS for Robot 1
    bridge_1 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/tb3_0/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/tb3_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/tb3_0/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/tb3_0/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen',
        remappings=[
            ('/model/tb3_0/cmd_vel', '/tb3_0/cmd_vel'),
            ('/model/tb3_0/odometry', '/tb3_0/odom'),
            ('/model/tb3_0/scan', '/tb3_0/scan'),
            ('/model/tb3_0/imu', '/tb3_0/imu'),
        ]
    )
    
    # Bridge topics from Gazebo to ROS for Robot 2
    bridge_2 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/tb3_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/tb3_1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/tb3_1/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/tb3_1/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen',
        remappings=[
            ('/model/tb3_1/cmd_vel', '/tb3_1/cmd_vel'),
            ('/model/tb3_1/odometry', '/tb3_1/odom'),
            ('/model/tb3_1/scan', '/tb3_1/scan'),
            ('/model/tb3_1/imu', '/tb3_1/imu'),
        ]
    )
    
    # SLAM Toolbox configuration
    slam_share = get_package_share_directory("slam_toolbox")
    slam_params = os.path.join(slam_share, "config", "mapper_params_online_async.yaml")
    
    # SLAM for Robot 1
    slam_node_1 = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="tb3_0",
        output="screen",
        parameters=[
            slam_params,
            {
                "use_sim_time": use_sim_time,
                "odom_frame": "tb3_0/odom",
                "map_frame": "tb3_0/map",
                "base_frame": "tb3_0/base_footprint",
                "scan_topic": "/tb3_0/scan"
            }
        ]
    )
    
    # SLAM for Robot 2
    slam_node_2 = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="tb3_1",
        output="screen",
        parameters=[
            slam_params,
            {
                "use_sim_time": use_sim_time,
                "odom_frame": "tb3_1/odom",
                "map_frame": "tb3_1/map",
                "base_frame": "tb3_1/base_footprint",
                "scan_topic": "/tb3_1/scan"
            }
        ]
    )
    
    # Lifecycle managers
    lifecycle_manager_1 = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        namespace='tb3_0',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['slam_toolbox'],
        }],
    )
    
    lifecycle_manager_2 = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        namespace='tb3_1',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['slam_toolbox'],
        }],
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        gazebo_launch,
        robot_state_publisher_1,
        robot_state_publisher_2,
        spawn_robot1,
        spawn_robot2,
        bridge_1,
        bridge_2,
        slam_node_1,
        slam_node_2,
        lifecycle_manager_1,
        lifecycle_manager_2,
    ])
