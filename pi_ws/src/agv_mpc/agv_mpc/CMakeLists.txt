cmake_minimum_required(VERSION 3.5)
project(agv_mpc)

add_definitions(-std=c++11 -O3)

option(BUILD_EXAMPLE "Whether or not building the CppAD & Ipopt example" OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
)

# ADD THIS
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(/usr/lib)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES agv_mpc
   CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf visualization_msgs
)

###########
## Build ##
###########

add_executable(
  tracking_reference_trajectory
  src/trackRefTraj.cpp
  src/trackRefTrajNode.cpp
)

target_link_libraries(
  tracking_reference_trajectory
  ipopt
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
#############
## Example ##
#############

if(BUILD_EXAMPLE)

    add_executable(
      CppAD_started
      example/CppAD_started.cpp
    )

    target_link_libraries(CppAD_started)

    add_executable(
      CppAD_Ipopt
      example/CppAD_Ipopt.cpp
    )

    target_link_libraries(
      CppAD_Ipopt
      ipopt
    )

endif()