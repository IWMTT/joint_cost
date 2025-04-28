cmake_minimum_required(VERSION 3.0.2)
project(joint_cost)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  rospy
  std_msgs
  message_generation
  message_runtime
  rospy
  roscpp
  std_msgs
  kdl_parser
  sensor_msgs
  geometry_msgs
  tf
  tf2
  roslib
  urdf
)

catkin_python_setup()


catkin_package(

)



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

