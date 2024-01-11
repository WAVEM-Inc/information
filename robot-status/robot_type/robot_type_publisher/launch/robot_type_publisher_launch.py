
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_type_param_dir = LaunchConfiguration(
        'robot_type_publisher',
        default=os.path.join(
            get_package_share_directory('robot_type_publisher'),
            'param',
            'atc.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_type_publisher',
            default_value=robot_type_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        Node(
            package='robot_type_publisher',
            executable='robot_type_publisher_node',
            parameters=[robot_type_param_dir],
            output='screen'),
    ])
