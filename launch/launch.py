from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mc_sim',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[{'frequency': 100}] 
        ),
        Node(
            package='mc_sim',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[{'frequency': 40}] 
        ),
        Node(
            package='mc_sim',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{'frequency': 20}]
        ),
        Node(
            package='mc_sim',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[
                {'lidar_processing_time_ms': 100},
                {'imu_processing_time_ms': 200},
                {'camera_processing_time_ms': 400}
            ]
        ),
        # Node(
        #     package='mc_sim',
        #     executable='multi_nav_node',
        #     name='multi_nav_node',
        #     output='screen',
        #     parameters=[
        #         {'lidar_processing_time_ms': 100},
        #         {'imu_processing_time_ms': 100},
        #         {'camera_processing_time_ms': 100}
        #     ]
        # ),
    ])
