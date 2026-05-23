cmake_minimum_required(VERSION 3.8)
project(ground_agent)

# Enable Python-based installation
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(python_cmake_module REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(shared_interfaces REQUIRED)
find_package(shared_infrastructure REQUIRED)

# Install the Python module
ament_python_install_package(${PROJECT_NAME})

# Install the shell wrapper for ROS2 CLI to find
install(PROGRAMS
  scripts/mission_handler_node
  scripts/navigation_node
  scripts/dust_plume_hazard_orbit_twist_commander_node
  scripts/path_planner_node
  scripts/on_arrival_task_node
  scripts/initial_inspection_node
  scripts/dust_sensor_relay_node
  scripts/fibrous_hazard_orbit_twist_commander_node
  scripts/dust_plume_density_estimation_node
  
  scripts/tf_listener_node
  scripts/tf_publisher_node
  scripts/twist_relay_node
  scripts/gps_covariance_injector_node
  scripts/imu_covariance_injector_node
  scripts/wait_for_tf_node
  

  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}
)

# Install models
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

# Install ai models
install(DIRECTORY ai_models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)


# Install all .py launch files from the launch/ directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDFs
install(DIRECTORY urdfs
  DESTINATION share/${PROJECT_NAME}
)

# Install config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# # --- MoveIt configs & launches ---------------------------------
# install(
#   DIRECTORY moveit            # ← whole folder, no trailing slash
#   DESTINATION share/${PROJECT_NAME}
# )



message(STATUS "✅ Ground Agent CMakeLists.txt is running")


ament_export_dependencies(rosidl_default_runtime)
ament_package()

