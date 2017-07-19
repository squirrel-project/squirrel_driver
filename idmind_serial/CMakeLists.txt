cmake_minimum_required(VERSION 2.8.3)
project(idmind_serial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp serial
  INCLUDE_DIRS include
  LIBRARIES idmind_serial
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(idmind_serial src/${PROJECT_NAME}/idmind_serial.cpp)

add_dependencies(idmind_serial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(idmind_serial ${catkin_LIBRARIES})

###########
## Install ##
###########
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS idmind_serial
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
