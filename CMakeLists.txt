cmake_minimum_required(VERSION 3.0.2)
project(sonar_array)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  serial
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sonar_array
#  CATKIN_DEPENDS roscpp rospy sensor_msgs serial std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sonar_reader_node 
  src/sonar_reader_node.cpp
  src/sonar_reader.cpp
  )
target_link_libraries(sonar_reader_node ${catkin_LIBRARIES})
