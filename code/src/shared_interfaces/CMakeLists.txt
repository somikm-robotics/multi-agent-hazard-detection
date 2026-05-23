cmake_minimum_required(VERSION 3.8)
project(shared_interfaces)

# Interface generator
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/OverrideMission.srv"
  "srv/ScanIncident.srv"
  "srv/ClassifyHazard.srv"
  "srv/DetectHazard.srv"
  "srv/DetectToxicGas.srv"
  "srv/RequestPathPlan.srv"
  "srv/PathPlanResult.srv"
  "srv/OrbitHazard.srv"
  "srv/NavigateRequest.srv"
  "srv/InitialInspection.srv"
  "srv/PlumeDensity.srv"
    
  "msg/ToxicityResult.msg"
  "msg/DustSensorResult.msg"

  "msg/HazardPose.msg"
  "msg/NavigationResult.msg"
  "msg/BaseReturn.msg"
  "msg/OnArrivalTask.msg"
  "msg/MissionStatus.msg"
  "msg/UIOverrideMission.msg"
  "msg/DensityEstimationResult.msg"

  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Optional: install the .srv files too
install(DIRECTORY srv msg
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

