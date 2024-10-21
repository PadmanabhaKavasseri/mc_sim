from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mc_sim',
            executable='lidar_node',
            name='lidar_node',
            output='screen'
        ),
        Node(
            package='mc_sim',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),
    ])
