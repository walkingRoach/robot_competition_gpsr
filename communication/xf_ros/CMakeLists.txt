cmake_minimum_required(VERSION 2.8.3)
project(xf_ros)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  message_generation
)

find_package(TinyXML REQUIRED)

add_action_files(
  DIRECTORY
        action
  FILES
    TTS.action
    HomeRecognize.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
)

catkin_package(
INCLUDE_DIRS
        include
LIBRARIES
    ${PROJECT_NAME}_lib
CATKIN_DEPENDS
    rospy
    roscpp
    std_msgs
    actionlib_msgs
)

include_directories(
        include
  ${TinyXML_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(lib)
add_library(${PROJECT_NAME}_lib
        src/asr.cpp
        src/xf/linuxrec.c
        src/xf/speech_recognizer.c
        )

target_link_libraries(${PROJECT_NAME}_lib
        dl
        msc
        asound
        pthread
        stdc++
        ${TinyXML_LIBRARIES}
        ${catkin_LIBRARIES}
        )

add_executable(asr_node src/asr_node.cpp)
add_executable(tts_node src/tts_node.cpp)
add_executable(awaken_node src/awaken.cpp)

target_link_libraries(asr_node ${PROJECT_NAME}_lib)
target_link_libraries(tts_node ${PROJECT_NAME}_lib)
target_link_libraries(awaken_node ${PROJECT_NAME}_lib)
