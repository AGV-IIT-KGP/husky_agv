cmake_minimum_required(VERSION 3.0.3)
project(outdoor_waypoint_nav)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  roslib
  roslaunch
)

catkin_package()
roslaunch_add_file_check(launch)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(switchgps src/switchgps.cpp)
target_link_libraries(switchgps ${catkin_LIBRARIES})

add_executable(switchgps_old src/switchgps_old.cpp)
target_link_libraries(switchgps_old ${catkin_LIBRARIES})

install(DIRECTORY  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
