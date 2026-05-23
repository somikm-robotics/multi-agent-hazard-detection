from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    
    shutdown_node = Node(
        package='shared_infrastructure',
        executable='shutdown_trigger_node',
        output='screen'
    )

    # Register handler that watches for shutdown_trigger_node to exit
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=shutdown_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason='🔴 Shutdown command received from GUI'))
            ]
        )
    )

    return LaunchDescription([
        shutdown_node,
        shutdown_handler
    ])
