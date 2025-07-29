from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='True',
        description='Whether to launch the GUI for sliders'
    )

    use_gui = LaunchConfiguration('use_gui')

    gui_node = Node(
        package='slider_gui',
        executable='gui_node',
        name='slider_gui_node',
        output='screen',
        parameters=[{'use_gui': use_gui}],
        emulate_tty=True
    )

    controller_node = Node(
        package='odrive_controller',
        executable='odrive_controller_node',
        name='odrive_controller_node',
        output='screen',
        emulate_tty=True
    )

    # Shutdown the whole launch if the GUI exits
    shutdown_on_gui_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=gui_node,
            on_exit=[Shutdown(reason='GUI node exited')]
        )
    )

    return LaunchDescription([
        use_gui_arg,
        gui_node,
        controller_node,
        shutdown_on_gui_exit
    ])
