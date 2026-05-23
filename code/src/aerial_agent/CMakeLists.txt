cmake_minimum_required(VERSION 3.8)
project(aerial_agent)

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
 
  scripts/crazyflie_patrol_node
  scripts/hazard_detection_node
  scripts/notifier_node
  scripts/tf_publisher_node
  scripts/toxicity_measurement_node

  DESTINATION lib/${PROJECT_NAME}
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


# Install config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)



message(STATUS "✅ Ground Agent CMakeLists.txt is running")


ament_export_dependencies(rosidl_default_runtime)
ament_package()

