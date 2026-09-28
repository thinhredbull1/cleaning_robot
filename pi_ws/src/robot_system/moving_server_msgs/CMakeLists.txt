cmake_minimum_required(VERSION 3.0.2)

project(moving_server_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
  message_generation
)

add_message_files(
  FILES
  MovingServerState.msg
)

add_service_files(
  FILES
  DetectDock.srv
)

add_action_files(
  FILES
  MoveCommand.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
  message_runtime
)
