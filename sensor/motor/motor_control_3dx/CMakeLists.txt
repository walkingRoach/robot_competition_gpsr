cmake_minimum_required(VERSION 2.8.3)
project(motor_control_3dx)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  rospy
  std_msgs
  message_generation
  tf
  rosbag
  serial
)


 add_message_files(
   FILES
   readDataAll.msg
   multi_range.msg
 )


 generate_messages(
   DEPENDENCIES
   sensor_msgs#   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES motor_control
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
  ${catkin_INCLUDE_DIRS}
)


add_executable(joystick src/joystick.cpp)
target_link_libraries(joystick
   ${catkin_LIBRARIES}
)

add_executable(motor_driver_3dx src/motor_driver_3dx.cpp)
target_link_libraries(motor_driver_3dx
   ${catkin_LIBRARIES}
)

