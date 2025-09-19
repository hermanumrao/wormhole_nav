from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # AMCL node (requires a map)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # Nav2 bringup (optional, adjust as needed)
        Node(
            package='nav2_bringup',
            executable='bringup_launch',
            name='nav2_bringup'
        ),
        # Our navigation node
        Node(
            package='simple_navigation',
            executable='simple_nav_node',
            name='simple_nav_node',
            output='screen'
        )
    ])
