cmake_minimum_required(VERSION 2.8.3)
project(nav_ros)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        roscpp
        rospy
        sensor_msgs
        std_msgs
        tf
        )

# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )


include_directories(
        ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )