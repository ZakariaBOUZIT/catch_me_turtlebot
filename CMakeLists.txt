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
 
 add_executable(main src/main.cpp)
 target_link_libraries(main
   ${catkin_LIBRARIES}
 )
 
  add_executable(pose_srv src/pose_srv.cpp)
 target_link_libraries(pose_srv
   ${catkin_LIBRARIES}
 )
 
  add_executable(turtlebot_move src/turtlebot_move.cpp)
 target_link_libraries(turtlebot_move
   ${catkin_LIBRARIES}
 )
 
   add_executable(scan_move src/scan_move.cpp)
 target_link_libraries(scan_move
   ${catkin_LIBRARIES}
 )
