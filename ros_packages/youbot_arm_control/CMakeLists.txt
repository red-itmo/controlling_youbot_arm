cmake_minimum_required(VERSION 2.8.3)
project(youbot_arm_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  brics_actuator

  message_generation
)

add_service_files(
  FILES
  IK.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

#catkin_package(
#    CATKIN_DEPENDS rospy std_msgs
#)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(
    control_node
    src/control_node.cpp
    src/ControlNode.cpp
)

target_link_libraries(
    control_node
    ${catkin_LIBRARIES}
)
