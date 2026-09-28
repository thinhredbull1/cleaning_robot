cmake_minimum_required(VERSION 3.0.2)

project(moving_server)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  tf2_ros
  tf2_geometry_msgs
  moving_server_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  tf2_ros
  tf2_geometry_msgs
  moving_server_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/moving_server_node.py
    scripts/odom_reset_node.py
    scripts/mock_dock_detector_node.py
    scripts/move_command_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
