cmake_minimum_required(VERSION 2.8.3)
project(uibk_robot_driver)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  control_toolbox
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs control_toolbox
  DEPENDS dynamixel_lib rt
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(dynamixel_lib
  src/dynamixel_sdk/GroupBulkRead.cpp
  src/dynamixel_sdk/GroupBulkWrite.cpp
  src/dynamixel_sdk/GroupSyncRead.cpp 
  src/dynamixel_sdk/GroupSyncWrite.cpp
  src/dynamixel_sdk/PacketHandler.cpp 
  src/dynamixel_sdk/PortHandler.cpp 
  src/dynamixel_sdk/Protocol1PacketHandler.cpp 
  src/dynamixel_sdk/Protocol2PacketHandler.cpp 
  src/dynamixel_sdk/PortHandlerLinux.cpp)

add_library(uibk_robot_driver
  src/base_controller.cpp
  src/arm_controller.cpp
  src/motor.cpp
  src/robot_controller.cpp
)

add_executable(uibk_arm_demo src/uibk_arm_demo.cpp)
add_executable(robot_controller_node src/robot_controller_node.cpp)
add_executable(arm_controller_sim src/ArmControllerSim.cpp)

add_dependencies(uibk_robot_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} dynamixel_lib)
add_dependencies(uibk_arm_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} uibk_robot_driver)
add_dependencies(robot_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} uibk_robot_driver)
add_dependencies(arm_controller_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} uibk_robot_driver)

target_link_libraries(uibk_robot_driver
  ${catkin_LIBRARIES}
  dynamixel_lib
  rt
)

target_link_libraries(uibk_arm_demo
  ${catkin_LIBRARIES}
  uibk_robot_driver
)

target_link_libraries(robot_controller_node
  ${catkin_LIBRARIES}
  uibk_robot_driver
)

target_link_libraries(arm_controller_sim
  ${catkin_LIBRARIES}
  uibk_robot_driver
)

FILE(GLOB_RECURSE LibFiles "*.h*")
add_custom_target(headers SOURCES ${LibFiles})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/dynamixel_sdk
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".git" EXCLUDE
)

install(TARGETS 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
