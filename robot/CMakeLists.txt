cmake_minimum_required(VERSION 2.8.3)
project(robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  facenet_ros
  roscpp
)

catkin_package(
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

