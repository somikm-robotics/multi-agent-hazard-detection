#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from shared_infrastructure.utils import get_agilex_base_pose

def generate_launch_description():

    pkg_share = get_package_share_directory('ground_agent')

  
    qos_sensor = {
        'qos_overrides./gps/fix.subscription.reliability': 'best_effort',
        'qos_overrides./gps/fix_with_cov.subscription.reliability': 'best_effort',
        'qos_overrides./imu.subscription.reliability': 'best_effort',
        'qos_overrides./odometry/local.subscription.reliability':    'best_effort',  
         # ★ global EKF needs this:
        'qos_overrides./odometry/gps.subscription.reliability':       'best_effort',
    }

    gps_tf_pub = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gps_tf_pub",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0.05",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "agilex_diff_drive/base_link",
            "--child-frame-id", "agilex_diff_drive/base_link/GazeboNavSat"
        ],
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    imu_tf_pub = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_tf_pub",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0.068",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "agilex_diff_drive/base_link",
            "--child-frame-id", "agilex_diff_drive/base_link/imu_sensor"
        ],
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    gps_covariance_node = Node(
            package='ground_agent',
            executable='gps_covariance_injector_node',
            name='gps_covariance_injector',
            output='screen',
            parameters=[{'use_sim_time': True}, qos_sensor]
    )
    
    imu_covariance_injector_node = Node(
        package='ground_agent',
        executable='imu_covariance_injector_node',
        name='imu_covariance_injector',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    xyzY = get_agilex_base_pose()
    map_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_transform',
        output='screen',
        arguments=[
            '--x', str(xyzY[0]),
            '--y', str(xyzY[1]),
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
    )
    
    static_tf_prefixed_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_prefixed_base',
        arguments=['0','0','0','0','0','0','base_link','agilex_diff_drive/base_link'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    static_tf_prefixed_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_prefixed_lidar',
        arguments=['0','0','0','0','0','0',
                'agilex_diff_drive/base_link','agilex_diff_drive/base_link/gpu_lidar'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )


    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[{
            "magnetic_declination_radians": 0.0,
            "yaw_offset": 0.0,
            "zero_altitude": True,
            "use_odometry_yaw": True,
            "wait_for_datum": False,
            "publish_filtered_gps": False,
            "broadcast_utm_transform": False,
            "use_simtime": True,
            "print_diagnostics": False,
        }],
        remappings=[
            ("imu/data", "/agilex/imu/data/with_cov"),
            ("gps/fix", "/gps/fix/with_cov"),
            ("gps/filtered", "/gps/filtered"),
            ("odometry/gps", "/odometry/gps"),
            ("odometry/filtered", "/odom")
        ],
    )

    ukf_localization_node = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_node',
        output='screen',
        respawn=True,
        parameters=[os.path.join(pkg_share, 'config/ukf_agilex.yaml')],
        remappings=[
            ("odometry/filtered", "/odom"),
            ("odometry/gps", "/odometry/gps")
        ]
        )

    return LaunchDescription(
        [
            ukf_localization_node,
            navsat_transform_node,
            gps_tf_pub,
            imu_tf_pub,  
            imu_covariance_injector_node,      
            gps_covariance_node,
            static_tf_prefixed_base,
            static_tf_prefixed_lidar,
            map_transform_node
            

        ]
    )


