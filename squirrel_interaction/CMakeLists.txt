cmake_minimum_required(VERSION 2.8.3)
project(squirrel_interaction)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
)

catkin_python_setup()

add_service_files(FILES
  GetList.srv
  StopMotor.srv
  StartMotor.srv
  DisplayScreen.srv
  PlaySound.srv
  DoorController.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS 
    src/squirrel_interaction/board/board.py src/squirrel_interaction/teleop.py src/squirrel_interaction/screen/raspberry_screen_server.py src/squirrel_interaction/speakers/play_sound.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
