cmake_minimum_required(VERSION 2.8.3)
project(dogwalker_core)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package()

install(PROGRAMS
  scripts/controller.py
  scripts/web_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
