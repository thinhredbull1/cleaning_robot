cmake_minimum_required(VERSION 3.0.2)

project(localize_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  map_server
  message_generation
)

add_service_files(
  FILES
  StartMapping.srv
  SaveMap.srv
  StartLocalization.srv
  Stop.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  std_srvs
  map_server
  message_runtime
)

catkin_install_python(
  PROGRAMS
    scripts/localize_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)