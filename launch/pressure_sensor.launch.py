from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pressure_sensor',
            executable='pressure_node',
            name='pressure_node',
            output='screen'  
        ),
        Node(
            package='pressure_sensor',
            executable='depth_odom_node',
            name='depth_odom_node',
            output='screen'  
        )
    ])
