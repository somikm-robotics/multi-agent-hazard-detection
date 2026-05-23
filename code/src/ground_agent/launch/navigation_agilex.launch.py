#/bin/python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

use_sim_time = LaunchConfiguration('use_sim_time', default=True)
def generate_launch_description():
    
    # make sure the arg exists (harmless if parent already declared it)
    declare_sim_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (/clock) time'
    )
    
    pkg_share = get_package_share_directory('ground_agent')


    params_file_name = "nav2_params_agilex.yaml"

    # Update map file path in params file
    params_file = os.path.join(pkg_share, 'config', params_file_name)

    map_file = os.path.join(pkg_share, 'worlds', 'mining_terrain_map.yaml')
    param_substitutions = {
        'yaml_filename': map_file
        }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)


    # Start map server
    map_server_node = Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                # parameters=[configured_params],
                parameters = [
                    {
                        'yaml_filename': str(map_file),
                        'topic_name':    '/map',        # default
                        'frame_id':      'map',
                        'use_sim_time':  True
                    }
                ],
                arguments=['--ros-args', '--log-level', 'info'])

    map_server_lifecycle_node = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map',
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
                parameters=[{'use_sim_time': True},
                            {'autostart': True},
                            {'node_names': ['map_server']}
                            ])


    
    
    twist_relay_node = Node(
        package='ground_agent',  
        executable='twist_relay_node',  # Make sure this script is installed
        name='twist_relay_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')


    # Start navigation
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments=[
            ('map', ''),
            ("slam", "False"), 
            ('use_sim_time', "True"),
            {'autostart', "True" },
            ('params_file',  str(params_file)),
            ('wait_for_transform',   'True'),   # <— Nav2 waits until map→base_link exists
            ('transform_timeout',    '3.0'),

            # ─────────────── Controller ───────────────
            ('controller_server.launch_ros_arguments',
            '"--ros-args '
            '--log-level controller_server:=debug '
            '--log-level nav2_controller:=debug '
            '--log-level dwb_core:=debug"'),

            # ─────────────── Planner ──────────────────
            ('planner_server.launch_ros_arguments',
            '"--ros-args '
            '--log-level planner_server:=debug '
            '--log-level nav2_planner:=debug '
            '--log-level nav2_navfn_planner:=debug"'),

            # ─────────────── Behaviors ────────────────
            ('behavior_server.launch_ros_arguments',
            '"--ros-args '
            '--log-level behavior_server:=debug '
            '--log-level nav2_behaviors:=debug"'),

            # ─────────────── BT Navigator ─────────────
            ('bt_navigator.launch_ros_arguments',
            '"--ros-args '
            '--log-level bt_navigator:=debug '
            '--log-level nav2_bt_navigator:=debug"'),

            # ─────────────── Local cost-map ───────────
            ('local_costmap.launch_ros_arguments',
            '"--ros-args '
            '--log-level local_costmap.local_costmap:=debug '
            '--log-level nav2_costmap_2d:=debug"'),

            # ─────────────── Global cost-map ──────────
            ('global_costmap.launch_ros_arguments',
            '"--ros-args '
            '--log-level global_costmap.global_costmap:=debug '
            '--log-level nav2_costmap_2d:=debug"'),
        ]
    )
  
    return LaunchDescription(
        [
            declare_sim_arg,
            SetEnvironmentVariable(
                name='GZ_SIM_SYSTEM_PLUGIN_PATH',
                value='/root/ros2_ws/install/lib'
            ),
            SetEnvironmentVariable(
                name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
                value='/root/ros2_ws/install/lib'
            ),
            
            map_server_node,
            map_server_lifecycle_node,
               twist_relay_node,
            nav2_bringup_launch
        ]
    )


