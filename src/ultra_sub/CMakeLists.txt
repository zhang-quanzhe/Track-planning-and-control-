cmake_minimum_required(VERSION 3.0.2)
project(ultra_sub)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ultra_sub
  CATKIN_DEPENDS roscpp rospy std_msgs
 DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ultra_sub src/ultra_sub.cpp)
target_link_libraries(ultra_sub ${catkin_LIBRARIES})

