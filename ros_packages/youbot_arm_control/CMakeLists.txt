cmake_minimum_required(VERSION 2.8.3)
project(youbot_arm_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
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

catkin_package(
    CATKIN_DEPENDS rospy std_msgs
)

catkin_package()
