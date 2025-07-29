# File: bparm_launches/launch/fk_demo.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='link6_fk',
            executable='fk_node',
            name='fk_node',
            output='screen'
        ),
    ])
