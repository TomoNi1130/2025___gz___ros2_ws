import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotSubstitution
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    bringup_dir = get_package_share_directory("bring_up")
    world_dir = os.path.join(bringup_dir, "world")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world",default_value=os.path.join(world_dir, "2025_field.sdf.xml"),description="Full path to world model file to load",),
        DeclareLaunchArgument("robot_name", default_value="robot", description="name of the robot"),
        DeclareLaunchArgument("left_lidar_topic", default_value="left_lidar", description="name of the left lidar topic"),
        DeclareLaunchArgument("right_lidar_topic", default_value="right_lidar", description="name of the right lidar topic"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    robot_name = LaunchConfiguration("robot_name")

    robot_frame_id = 'robot_base'
    left_lidar_frame_id = 'left_lidar'
    right_lidar_frame_id = 'right_lidar'

    left_lidar_topic = "left_lidar"
    right_lidar_topic = "right_lidar"
    merged_lidar_topic = "merged_points"

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(bringup_dir, "config", "2025_rviz2_config.rviz")],
    )

    gz_sim = ExecuteProcess(
        cmd=["gz","sim","-v4","-r",world,],
        name="gz_sim",
        output="screen",
    )

    gz_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-x",
            "0.65",
            "-y",
            "4.75",
            "-z",
            "0.3",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "robot_description.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    moter_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="moter_bridge",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "override_timestamps_with_wall_time": NotSubstitution(use_sim_time),
            }
        ],
        arguments=[
            "/FR_v@std_msgs/msg/Float64@gz.msgs.Double",
            "/FL_v@std_msgs/msg/Float64@gz.msgs.Double",
            "/BR_v@std_msgs/msg/Float64@gz.msgs.Double",
            "/BL_v@std_msgs/msg/Float64@gz.msgs.Double",
        ],
    )

    lidar_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="lidar_bridge",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "override_timestamps_with_wall_time": NotSubstitution(use_sim_time),
            }
        ],
        arguments=[
            f"/{left_lidar_topic}@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            f"/{right_lidar_topic}@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
        ],
    )

    basic_run = ComposableNodeContainer(
        name='basic_run_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='bring_up',
                plugin='UC::StaticTF',
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[{
                    'use_sim_time' : use_sim_time,
                    'robot_frame_id': robot_frame_id,
                    'left_lidar_frame_id': f'robot/{robot_frame_id}/{left_lidar_frame_id}',
                    'right_lidar_frame_id': f'robot/{robot_frame_id}/{right_lidar_frame_id}',
                }],
            ),
        ]
    )

    delayed_load_1 = TimerAction(
        period=1.0,
        actions=[
            ComposableNodeContainer(
                name='points_processer_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                output='screen',
                composable_node_descriptions=[
                    ComposableNode(
                    package='points_processes',
                    plugin='points_processes::PointIntegration',

                    extra_arguments=[{'use_intra_process_comms': True,}],
                    parameters=[{
                        'use_sim_time' : use_sim_time,
                        'scan_topic_names': [left_lidar_topic, right_lidar_topic],
                        'merged_topic_name': merged_lidar_topic,
                        'merged_frame_id': robot_frame_id,
                    }],
                ),]
            ),
        ]
    )

    return LaunchDescription(
        args + 
        [
        rviz,
        gz_sim,gz_spawn_robot,robot_description,
        moter_bridge,lidar_bridge,
        basic_run,
        delayed_load_1,
        ]
    )