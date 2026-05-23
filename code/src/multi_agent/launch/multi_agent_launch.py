# agilex_launch.py  (drop this under multi_agent/launch)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, Shutdown, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue
from math import pi
from shared_infrastructure.utils import get_current_hazard_type_and_pose, get_agilex_base_pose
from shared_infrastructure.hazard_types import HazardType
from shared_infrastructure.task_types import TaskType
from shared_infrastructure.utils import get_current_ore_spawn_mode_from_config, ore_spawn_type_to_str
from shared_infrastructure.ore_spawn_types import OreSpawnType


def generate_launch_description():

    curr_pkg = FindPackageShare('multi_agent')
    aerial_pkg = FindPackageShare('aerial_agent')
    ground_pkg = FindPackageShare('ground_agent')
    shared_infra_pkg = FindPackageShare('shared_infrastructure')
    material_handling_pkg = FindPackageShare('material_handling_agent')

    hazard_models_path = PathJoinSubstitution([curr_pkg, 'models', 'hazard_overlays'])

    rviz_config = PathJoinSubstitution([curr_pkg, 'rviz', 'multi_agent_with_image_nav2_view.rviz']
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='true',
    description='Use simulation clock (Gazebo)'
)

    set_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            # ground_agent models
            PathJoinSubstitution([ground_pkg, 'models']),
            TextSubstitution(text=':'),
            # aerial_agent models
            PathJoinSubstitution([aerial_pkg, 'models']),
            TextSubstitution(text=':'),
            PathJoinSubstitution([material_handling_pkg, 'models']),
            TextSubstitution(text=':'),
            # multi_agent (shared) models
            PathJoinSubstitution([curr_pkg, 'models']),
            TextSubstitution(text=':'),
            # multi_agent (hazard overlays) models
            PathJoinSubstitution([curr_pkg, 'models', 'hazard_overlays']),
            TextSubstitution(text=':'),
            # multi_agent (obstacles) models
            PathJoinSubstitution([curr_pkg, 'models', 'obstacles']),
            TextSubstitution(text=':'),
            # ← don’t crash if the variable isn’t set yet
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
        ],
    )

    set_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value='/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins'
        ':/root/ros2_ws/install/lib'
    )

    set_log_level = SetEnvironmentVariable(
        name='IGN_LOG_LEVEL', value='4')
    
    # Synchronizes Gazebo's simulation time with ROS 2.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )


    
    hazard_info = get_current_hazard_type_and_pose()
    hazard_type = hazard_info["Hazard_Type"]

    urdf_name = "agilex.urdf"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [ground_pkg, 'urdfs', urdf_name]             
            ),
        ]
    )

    robot_description = {
    'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description,
                    {'frame_prefix': 'agilex_diff_drive/'},
                    {'use_sim_time': True},
                    {'tf_buffer_duration': 5.0} 
                    ],
        output='screen'
    )


    gz_sim_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch', 'gz_sim.launch.py'])]
            ),
            launch_arguments=[('gz_args', [' -r -v 1 ',
            PathJoinSubstitution([curr_pkg,
                        'worlds', 'mining.world.sdf'])]
        )])
    

    rviz2_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config,
                    '--ros-args', '--log-level', 'warn'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
    )

    # agilex
    xyzY = get_agilex_base_pose()
    gz_spawn_agilex = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                '-name', 'agilex_diff_drive',
                '-x',      str(xyzY[0]),          # ← set pose here
                '-y',      str(xyzY[1]),
                '-z',      str(xyzY[2]),
                '-Y',      str(xyzY[3]),         # yaw 180 deg
                '-allow_renaming', 'true'],
        output='screen',
    )

    # print(f"robot desc: {robot_description}")
    
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'diff_drive_base_controller'],
        output='screen'
    )

    # bridges
    
    agilex_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                '/agilex/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                    # Camera image
                '/world/mining_world/model/agilex_diff_drive/link/base_link/sensor/rgb_cam/image@sensor_msgs/msg/Image[gz.msgs.Image',

                    # Camera info
                    '/world/mining_world/model/agilex_diff_drive/link/base_link/sensor/rgb_cam/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                ],
        output='screen'
    )

    # Crazyflie control bridges
    crazyflie_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/crazyflie/gazebo/command/twist@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/crazyflie/enable@std_msgs/msg/Bool@gz.msgs.Boolean',
            '/world/mining_world/model/crazyflie/model/vga_camera/link/vga_camera/sensor/down_cam/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/world/mining_world/model/crazyflie/model/vga_camera/link/vga_camera/sensor/down_cam/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/model/crazyflie/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
        ],
        output='screen'
    )

    # hazard models
    HAZARD_MODEL_PATHS = {
        HazardType.FIBROUS_HAZARD: PathJoinSubstitution([hazard_models_path, 'asbestos_marker', 'model.sdf']),
        HazardType.DUST_PLUME: PathJoinSubstitution([hazard_models_path, 'dust_plume_marker', 'model.sdf']),
        HazardType.TOXIC_GAS: PathJoinSubstitution([hazard_models_path, 'toxic_gas_marker', 'model.sdf']),
    }

    
    x, y, z = hazard_info["Hazard_Poses"]
    

    # === RESOLVE MODEL ===
    model_path = HAZARD_MODEL_PATHS[hazard_type]
    model_name = hazard_type.name.lower()  # Clean name for spawning

    gz_spawn_hazard = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', model_path,
            '-name', model_name,
            '-x', str(x),
            '-y', str(y),
            '-z', str(z),
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

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

    # delay localisation stack by 3 s to let /clock appear
    localisation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ground_pkg, '/launch/localization_agilex.launch.py']
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    delay_localisation = TimerAction(
        period=3.0,
        actions=[localisation_launch]
    )

    # ----  helper process: wait up to 15 s for map→odom TF, then exit ------------
    wait_for_tf_proc = ExecuteProcess(
        cmd=['ros2', 'run', 'ground_agent', 'wait_for_tf_node', 'map', 'odom', '15'],
        output='screen'
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ground_pkg, 'launch', 'navigation_agilex.launch.py'])
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    start_nav2_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_tf_proc,
            on_exit=[nav2_launch]
        )
    )

        
        
    return LaunchDescription([
        
        use_sim_time_arg,
        set_env,
        set_plugin_path,
        # set_log_level,
        clock_bridge,
        
        # gazebo / ros / bridge
        gz_sim_include,

        robot_state_publisher,
        gz_spawn_agilex,
        agilex_bridge,
        crazyflie_bridge,
        gz_spawn_hazard,
        


        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_agilex,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),

        
        # # # ① joint-trajectory finished  →
        # # # ② localisation stack starts →
        # # # ③ helper waits for TF       →
        # # # ④ Nav2 launches
        RegisterEventHandler(OnProcessExit(
            target_action=load_joint_trajectory_controller,
            on_exit=[delay_localisation, wait_for_tf_proc])),


        # start_nav2_handler,
        

        # rviz2_node,

        # Launch agents
    
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([aerial_pkg, 'launch', 'aerial_nodes_launch.py'])
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([ground_pkg, 'launch', 'ground_nodes_launch.py'])
            )
        ),
        


        shutdown_trigger_node,   # listener must be started
        cascade_shutdown,         # handler must follow the node
    ])
