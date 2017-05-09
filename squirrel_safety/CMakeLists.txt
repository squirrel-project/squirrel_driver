cmake_minimum_required(VERSION 2.8.3)
project(squirrel_safety)

find_package(catkin REQUIRED
  rospy
  squirrel_safety_msgs)

catkin_python_setup()

catkin_package(

)

install(PROGRAMS
   scripts/squirrel_safety_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
   launch/squirrel_safety.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
