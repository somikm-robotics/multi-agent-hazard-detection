cmake_minimum_required(VERSION 3.8)
project(material_handling_agent)

# Enable Python-based installation
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(python_cmake_module REQUIRED)

# Install the Python module
ament_python_install_package(${PROJECT_NAME})

# Install the shell wrapper for ROS2 CLI to find
install(PROGRAMS
scripts/conveyor_classifier_slow_scroll_node

DESTINATION lib/${PROJECT_NAME}
)



# Install all .py launch files from the launch/ directory
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}/
)

# Install worlds
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install models
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

# Install config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


message(STATUS "✅ Material Handling Agent CMakeLists.txt is running")


ament_package()
