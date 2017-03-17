cmake_minimum_required(VERSION 2.8.3)
project(flow_control_py)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

add_service_files(
   FILES
   FlowCommand.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_install_python(
  PROGRAMS 
  scripts/flow_control_001.py
  scripts/flow_control_002.py
  scripts/flow_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)