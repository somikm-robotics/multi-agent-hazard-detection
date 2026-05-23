#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # When a Bool(true) arrives on /shutdown_system -> shut down launch
    # shutdown_handler = RegisterEventHandler(
    #     OnTopicMessage(
    #         topic='/shutdown_system',
    #         msg_type=Bool,
    #         on_receive=[Shutdown()]      # sends SIGINT to every child process
    #     )
    # )

    # When a Bool(true) arrives on /shutdown_system -> shut down launch
    # 1. create the listener node
    shutdown_trigger_node = Node(
        package='shared_infrastructure',
        executable='shutdown_trigger_node',   # console-script entry
        name='shutdown_trigger_node',
        output='screen'
    )

    # 2. when that node exits → shut the whole tree down
    cascade_shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=shutdown_trigger_node,
            on_exit=[Shutdown(reason='remote /shutdown_system')]
        )
    )

    return LaunchDescription([
        
        Node(
            package='supervisory_agent',
            executable='mission_overrider_node',
            name='mission_overrider_node',
            output='screen'
        ),
        Node(
            package='supervisory_agent',
            executable='ui_coordinator.py',  
            name='ui_coordinator',
            output='screen'
        ),
        shutdown_trigger_node,
        cascade_shutdown
    ])
