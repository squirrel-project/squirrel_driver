cmake_minimum_required(VERSION 2.8.3)
project(ft17_driver)

find_package(catkin REQUIRED)

find_program(CMAKE_DPKG dpkg /bin /usr/bin /usr/local/bin) 
if(CMAKE_DPKG)
  exec_program(dpkg ARGS --print-architecture OUTPUT_VARIABLE CMAKE_DPKG_ARCH)
  if(CMAKE_DPKG_ARCH MATCHES "amd64")
    message(STATUS "++++++++++++++++++++++++++ DETECTED 64 bit +++++++++++++++++++++++++++++++++++")
    set(MYARCH "x86_64")
  else()
    message(STATUS "++++++++++++++++++++++++++ DETECTED 32 bit +++++++++++++++++++++++++++++++++++")
    set(MYARCH "i686")
  endif()
endif()

catkin_package(
    INCLUDE_DIRS common/include
    LIBRARIES FT17_driver
)

include_directories(common/include)

file(COPY common/lib/${MYARCH}/libFT17_driver.so DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION})
file(COPY common/lib/${MYARCH}/libFT17_driver.so.0.0.1 DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION})

install(PROGRAMS common/lib/${MYARCH}/libFT17_driver.so.0.0.1 common/lib/${MYARCH}/libFT17_driver.so
  DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION}
)

install(DIRECTORY common/include/FT17
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(DIRECTORY common doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
