cmake_minimum_required(VERSION 2.8.3)
project(idmind_interaction)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  idmind_serial
  message_generation
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

add_message_files(
   FILES
   MotorControl.msg
   LedControl.msg
   LedControlBase.msg
   LedControlMouth.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(idmind_interaction src/idmind_interaction.cpp)
add_dependencies(idmind_interaction idmind_serial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(idmind_interaction ${catkin_LIBRARIES})