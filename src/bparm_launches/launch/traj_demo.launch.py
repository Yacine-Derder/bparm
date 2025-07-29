# File: bparm_launches/launch/traj_demo.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    args = [
        DeclareLaunchArgument('x', default_value='0.4'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.4'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        # Optionally add more args for start joint positions
    ]

    cmd_args = [
        LaunchConfiguration('x'),
        LaunchConfiguration('y'),
        LaunchConfiguration('z'),
        LaunchConfiguration('roll'),
        LaunchConfiguration('pitch'),
        LaunchConfiguration('yaw'),
        # Example: hardcoded 6 joint values (or use DeclareLaunchArgument for each)
        '0.0', '0.0', '0.0', '0.0', '0.0', '0.0'
    ]

    node = Node(
        package='trajectory_printer',
        executable='start_to_end_node',
        name='start_to_end_node',
        output='screen',
        arguments=cmd_args
    )

    return LaunchDescription(args + [node])
