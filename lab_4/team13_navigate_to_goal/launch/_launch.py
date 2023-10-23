from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='team13_navigate_to_goal',
            namespace='team_13_navigate_to_goal',
            executable='getObjectRange',
            name='obj_range'
        ),
        Node(
            package='team13_navigate_to_goal',
            namespace='team_13_navigate_to_goal',
            executable='goToGoal',
            name='driver'
        ),
    ])

