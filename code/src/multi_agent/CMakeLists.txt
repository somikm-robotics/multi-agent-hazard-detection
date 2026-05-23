cmake_minimum_required(VERSION 3.8)
project(multi_agent)

# Default to C++17 – matches ROS 2 Humble and the demo package
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Enable helpful warnings on GCC / Clang
if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)


# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install worlds
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install models
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Test files for Robots testing
add_executable(agilex_twist_nav_test tests/agilex_twist_nav_test.cpp)
ament_target_dependencies(agilex_twist_nav_test 
  rclcpp 
  geometry_msgs)

add_executable(crazyflie_twist_nav_test tests/crazyflie_twist_nav_test.cpp)
ament_target_dependencies(crazyflie_twist_nav_test 
  rclcpp 
  geometry_msgs
  std_msgs)


add_executable(agilex_hazard_follower_test tests/agilex_hazard_follower_test.cpp)
ament_target_dependencies(agilex_hazard_follower_test 
  rclcpp 
  rclcpp_action           # ✅ for sending Nav2 actions
  geometry_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
  nav2_msgs               # ✅ for NavigateToPose
  nlohmann_json
)

target_include_directories(agilex_hazard_follower_test PRIVATE
  tests
)

target_link_libraries(agilex_hazard_follower_test
  nlohmann_json::nlohmann_json
)


add_executable(agilex_short_goal_test tests/agilex_short_goal_test.cpp)
ament_target_dependencies(agilex_short_goal_test 
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2
  tf2_geometry_msgs
  nav2_msgs
  visualization_msgs
  nav2_util
 )


add_executable(crazyflie_square_patrol_test tests/crazyflie_square_patrol_test.cpp)
ament_target_dependencies(crazyflie_square_patrol_test rclcpp geometry_msgs std_msgs)


install(TARGETS 
agilex_twist_nav_test 
agilex_hazard_follower_test
agilex_short_goal_test

crazyflie_twist_nav_test
crazyflie_square_patrol_test


DESTINATION lib/${PROJECT_NAME})





ament_package()
