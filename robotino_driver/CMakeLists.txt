cmake_minimum_required(VERSION 2.8.3)
project(robotino_driver)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  nav_msgs
  pcl_conversions
  pcl_ros
  robotino_msgs
  tf
  roscpp
  std_srvs
  dynamixel_controllers
  dynamixel_msgs
  geometry_msgs
  sensor_msgs
  squirrel_view_controller_msgs
  std_msgs
)

list(APPEND CMAKE_MODULE_PATH "/usr/local/robotino/api2/cmake")
find_package(RobotinoAPI2 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS nav_msgs robotino_msgs std_srvs squirrel_view_controller_msgs dynamixel_controllers dynamixel_msgs geometry_msgs image_transport pcl_conversions pcl_ros sensor_msgs std_msgs tf
)

include_directories(
  include/robotino_driver
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  robotino_driver
  src/robotino_node.cpp
  src/AnalogInputArrayROS.cpp
  src/BumperROS.cpp
  src/CameraROS.cpp
  src/CompactBHAROS.cpp
  src/ComROS.cpp
  src/DigitalInputArrayROS.cpp
  src/DigitalOutputArrayROS.cpp
  src/DistanceSensorArrayROS.cpp
  src/ElectricalGripperROS.cpp
  src/EncoderInputROS.cpp
  src/MotorArrayROS.cpp
  src/NorthStarROS.cpp
  src/OdometryROS.cpp
  src/OmniDriveROS.cpp
  src/PowerManagementROS.cpp
  src/RobotinoNode.cpp)
target_link_libraries(robotino_driver ${REC_ROBOTINO_API2_LIBRARY} ${catkin_LIBRARIES})
add_dependencies(robotino_driver robotino_msgs_gencpp)

add_executable(robotino_tilt_controller
  src/tilt_controller_node.cpp
  src/TiltControllerNode.cpp)
target_link_libraries(robotino_tilt_controller ${catkin_LIBRARIES})
add_dependencies(robotino_tilt_controller robotino_msgs_gencpp)

add_executable(
  robotino_pan_controller
  src/pan_controller_node.cpp
  src/PanControllerNode.cpp)
target_link_libraries(robotino_pan_controller ${catkin_LIBRARIES})
add_dependencies(robotino_pan_controller robotino_msgs_gencpp)

add_executable(
  squirrel_view_controller
  #src/view_controller_node.cpp
  src/ViewController.cpp
)
add_dependencies(squirrel_view_controller squirrel_view_controller_msgs_gencpp)

target_link_libraries(
  squirrel_view_controller
  ${catkin_LIBRARIES}
)

add_executable(
  robotino_odometry_node
  src/robotino_odometry_node.cpp
  src/ComROS.cpp
  src/OdometryROS.cpp
  src/RobotinoOdometryNode.cpp)
target_link_libraries(robotino_odometry_node ${REC_ROBOTINO_API2_LIBRARY} ${catkin_LIBRARIES})
add_dependencies(robotino_odometry_node robotino_msgs_gencpp)

add_executable(
  robotino_camera_node
  src/robotino_camera_node.cpp
  src/ComROS.cpp
  src/CameraROS.cpp
  src/RobotinoCameraNode.cpp)
target_link_libraries(robotino_camera_node ${REC_ROBOTINO_API2_LIBRARY} ${catkin_LIBRARIES})
add_dependencies(robotino_camera_node robotino_msgs_gencpp)

add_executable(
  robotino_mapping_node
  src/robotino_mapping_node.cpp
  src/MappingROS.cpp
  src/RobotinoMappingNode.cpp
  src/InitialPoseROS.cpp
  src/NavGoalROS.cpp
  src/ComROS.cpp)

target_link_libraries(robotino_mapping_node ${REC_ROBOTINO_API2_LIBRARY} ${catkin_LIBRARIES})
add_dependencies(robotino_mapping_node robotino_msgs_gencpp)
