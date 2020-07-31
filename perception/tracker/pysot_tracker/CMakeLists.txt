cmake_minimum_required(VERSION 2.8.3)
project(pysot_tracker)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Box.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)


