from launch import LaunchDescription
from launch_ros.actions import Node
from shared_infrastructure.utils import get_current_hazard_type_and_pose, parse_hazard_type

def generate_launch_description():
    
    return LaunchDescription([

        Node(
            package='ground_agent',
            executable='mission_handler_node',
            name='mission_handler_node',
            output='screen'
        ),
        Node(
            package='ground_agent',
            executable='navigation_node',
            name='navigation_node',
            output='screen'
        ),
        Node(
            package='ground_agent',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen'
        ),
        Node(
            package='ground_agent',
            executable='on_arrival_task_node',
            name='on_arrival_task_node',
            output='screen'
        ),
        Node(
            package='ground_agent',
            executable='initial_inspection_node',
            name='initial_inspection_node',
            output='screen'
        ),
        Node(
            package='ground_agent',
            executable='dust_sensor_relay_node',
            name='dust_sensor_relay_node',
            output='screen'
        ),
        Node(
            package='ground_agent',
            executable='dust_plume_density_estimation_node',
            name='dust_plume_density_estimation_node',
            output='screen'
        ),
        Node(
            package='ground_agent',
            executable='fibrous_hazard_orbit_twist_commander_node',
            name='fibrous_hazard_orbit_twist_commander_node',
            output='screen'
        ),
        Node(
            package='ground_agent',
            executable='tf_listener_node',
            name='tf_listener_node',
            output='screen'
        ),
        # Node(
        #     package='ground_agent',
        #     executable='tf_publisher_node',
        #     name='tf_publisher_node',
        #     output='screen'
        # )

        
    ])
