cmake_minimum_required(VERSION 2.8.3)
project(kw_arm)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs rospy std_msgs message_generation tf rosbag serial genmsg actionlib_msgs actionlib)

add_action_files(FILES
  MoveToPose.action
)

add_message_files(FILES
  target.msg
)
## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  actionlib_msgs
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  INCLUDE_DIRS include/kw_arm
  LIBRARIES my_ik_solver

)

###########
## Build ##
###########

include_directories(
        include ${catkin_INCLUDE_DIRS}
        include/kw_arm
        src
)

add_library(my_ik_solver
        src/my_ik_solver.cpp
)

add_dependencies(my_ik_solver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(my_ik_solver
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############


#############
## Testing ##
#############

add_executable(MoveToPose_server
        src/MoveToPose_server.cpp
)
add_dependencies(MoveToPose_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(MoveToPose_server
  my_ik_solver
  ${catkin_LIBRARIES}
)

add_executable(MoveToPose_client
        src/MoveToPose_client.cpp
)
add_dependencies(MoveToPose_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(MoveToPose_client
  ${catkin_LIBRARIES}
)

add_executable(FixedMotion
        src/FixedMotion.cpp
)
add_dependencies(FixedMotion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(FixedMotion
  my_ik_solver
  ${catkin_LIBRARIES}
)


add_executable(TargetPub
        src/TargetPub.cpp
)
add_dependencies(TargetPub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(TargetPub
  ${catkin_LIBRARIES}
)