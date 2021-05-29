cmake_minimum_required(VERSION 3.0.2)
project(cancontrol)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cancontrol
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)


include_directories(
   ${catkin_INCLUDE_DIRS} include
)
 
link_directories(
  ${catkin_LIB_DIRS} lib
)
add_executable(cancontrol src/cancontrol.cpp)
target_link_libraries(cancontrol ${catkin_LIBRARIES} controlcan
)

