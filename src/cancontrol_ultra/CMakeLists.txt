cmake_minimum_required(VERSION 3.0.2)
project(cancontrol_ultra)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cancontrol_ultra
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)


include_directories(
   ${catkin_INCLUDE_DIRS} include
)
 
link_directories(
  ${catkin_LIB_DIRS} lib
)
add_executable(cancontrol_ultra src/cancontrol_ultra.cpp
	)
target_link_libraries(cancontrol_ultra ${catkin_LIBRARIES} controlcan
)

add_executable(cancontrol_ultra_loop src/cancontrol_ultra_loop.cpp
	)
target_link_libraries(cancontrol_ultra_loop ${catkin_LIBRARIES} controlcan
)
