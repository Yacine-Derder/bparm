# File: bparm_launches/launch/bparm.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to launch RViz in demo.launch.py'
    )
    use_rviz = LaunchConfiguration('use_rviz')

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bparm_5_1_moveit_config'),
                'launch',
                'demo.launch.py'
            )
        ),
        launch_arguments={'use_rviz': use_rviz}.items()
    )

    fk_node = Node(
        package='link6_fk',
        executable='fk_node',
        name='fk_node',
        output='screen'
    )

    compute_traj_node = Node(
        package='compute_traj',
        executable='compute_traj_node',
        name='compute_traj_node',
        output='screen'
    )

    relay_traj_node = Node(
        package='relay_traj',
        executable='relay_traj_node',
        name='relay_traj_node',
        output='screen',
        emulate_tty=True  # Python node
    )

    odrive_controller_node = Node(
        package='odrive_controller',
        executable='odrive_controller_node',
        name='odrive_controller_node',
        output='screen',
        emulate_tty=True  # Python node
    )

    return LaunchDescription([
        use_rviz_arg,
        moveit_launch,
        fk_node,
        compute_traj_node,
        relay_traj_node,
        odrive_controller_node
    ])
