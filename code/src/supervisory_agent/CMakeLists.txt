cmake_minimum_required(VERSION 3.8)
project(supervisory_agent)

# Enable Python-based installation
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(python_cmake_module REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Install the Python module
ament_python_install_package(${PROJECT_NAME})

# Install the shell wrapper for ROS2 CLI to find
install(PROGRAMS
  scripts/alert_receiver_node
  scripts/mission_overrider_node
  scripts/ui_interface_node
  scripts/ui_coordinator.py
  
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}
)

# Install all .py launch files from the launch/ directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

message(STATUS "✅ Supervisory Agent CMakeLists.txt is running")


ament_package()
