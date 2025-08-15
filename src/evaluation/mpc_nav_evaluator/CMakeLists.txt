cmake_minimum_required(VERSION 3.0.2)
project(mpc_nav_evaluator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
)
