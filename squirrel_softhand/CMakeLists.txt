cmake_minimum_required(VERSION 2.8.3)
project(squirrel_softhand)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   std_srvs
   squirrel_manipulation_msgs)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

catkin_package()

add_library(qb_lib src/qbmove_communications.cpp)

add_executable(softhand_controller src/softhand_controller.cpp)
target_link_libraries(softhand_controller qb_lib ${catkin_LIBRARIES})
add_dependencies(softhand_controller squirrel_manipulation_msgs_gencpp)

install(TARGETS
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
