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

def generate_launch_description():
    bringup_dir = get_package_share_directory("bring_up")
    desc_dir = get_package_share_directory("description")
    world_dir = os.path.join(bringup_dir, "world")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world",default_value=os.path.join(world_dir, "2025_field.sdf.xml"),description="Full path to world model file to load",),
        DeclareLaunchArgument("robot_name", default_value="robot", description="name of the robot"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
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
            os.path.join(desc_dir, "launch", "robot.launch.py")
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

    lider_bridge = Node(
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
            "/left_lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/right_lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
        ],
    )

    controller_conection = Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_node',
    )
    
    controller_run = Node(
        package="controller_pkg",
        executable="manual",
    )

    return LaunchDescription(
        args + 
        [
        gz_sim,
        gz_spawn_robot,
        robot_description,
        moter_bridge,
        lider_bridge,
        controller_conection,
        controller_run,
        ]
    )