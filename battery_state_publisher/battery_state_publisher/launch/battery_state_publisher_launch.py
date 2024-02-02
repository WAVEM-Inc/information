from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
def generate_launch_description():
    battery_state_publisher = Node(
        package='battery_state_publisher',
        executable='battery_state_publisher_node',
        output='screen'
    )
    return LaunchDescription([battery_state_publisher])
