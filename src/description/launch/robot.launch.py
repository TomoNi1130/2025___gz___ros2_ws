import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # ''use_sim_time'' is used to  determine whether ros2 use simulation time provided by simulation environment (Gazebo).
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    models_dir = get_package_share_directory("description")
    urdf = os.path.join(models_dir, "urdf", "robot.urdf")

    # open the whole urdf_file_name file and read it content to robot_desc
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

        return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
       Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
    ])