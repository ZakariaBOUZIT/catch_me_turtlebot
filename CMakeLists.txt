cmake_minimum_required(VERSION 3.0.2)
project(catch_me)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


#this function must be called before declaring any targets with add_library() or add_executable()
catkin_package()


#include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(listener src/listener.cpp)
 target_link_libraries(listener
   ${catkin_LIBRARIES}
 )

add_executable(talker src/talker.cpp)
 target_link_libraries(talker
   ${catkin_LIBRARIES}
 )
