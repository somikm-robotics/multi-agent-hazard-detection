cmake_minimum_required(VERSION 3.8)
project(shared_infrastructure)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(python_cmake_module REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# for plugin
find_package(ignition-gazebo6 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(shared_interfaces REQUIRED)  
find_package(geometry_msgs REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

ament_python_install_package(${PROJECT_NAME})


install(DIRECTORY shared_infrastructure
  DESTINATION lib/python3.10/site-packages
)


# gas_emitter plugin
add_library(gas_emitter SHARED plugins/GasEmitterPlugin.cc)
ament_target_dependencies(gas_emitter ignition-gazebo6 rclcpp std_msgs)

add_library(gas_sensor SHARED plugins/GasSensorPlugin.cc)
ament_target_dependencies(gas_sensor ignition-gazebo6 rclcpp std_msgs shared_interfaces)

add_library(dust_plume_sensor SHARED plugins/DustPlumeSensorPlugin.cc)
ament_target_dependencies(dust_plume_sensor ignition-gazebo6 rclcpp std_msgs shared_interfaces)


add_executable(CapsulePoseBridge 
 plugins/CapsulePoseBridge.cc
)

ament_target_dependencies(CapsulePoseBridge
  rclcpp
  geometry_msgs
)

# Link to Ignition targets 
target_link_libraries(CapsulePoseBridge
  ignition-transport11::core
  ignition-msgs8::core
)

install(TARGETS CapsulePoseBridge DESTINATION lib/${PROJECT_NAME})

install(TARGETS gas_emitter 
                gas_sensor
                dust_plume_sensor
        LIBRARY DESTINATION lib)

install(PROGRAMS
  scripts/shutdown_trigger_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install all .py launch files from the launch/ directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

message(STATUS "✅ Shared Onfrastructure CMakeLists.txt is running")


ament_package()