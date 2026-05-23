from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # ── Static TF: map  ➜  down-facing camera ───────────────────────────
    static_tf_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_camera",
        #         x  y   z     roll pitch   yaw    parent   child
        arguments=["0", "0", "-0.01", "0", "1.5708", "0",
                "map", "crazyflie/vga_camera/vga_camera/down_cam"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )


    return LaunchDescription([
        static_tf_camera,
        Node(package='aerial_agent', executable='hazard_detection_node', output='screen'),
        Node(package='aerial_agent', executable='notifier_node', output='screen'),
        Node(package='aerial_agent', executable='crazyflie_patrol_node', output='screen'),
        Node(package='aerial_agent', executable='toxicity_measurement_node', output='screen'),
        Node(package='aerial_agent', executable='tf_publisher_node', output='screen')
        
    ])
