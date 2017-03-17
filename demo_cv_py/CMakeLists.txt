cmake_minimum_required(VERSION 2.8.3)
project(demo_cv_py)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
)
catkin_package()

install(PROGRAMS scripts/demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})