## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
project(bluerov_bringup)
find_package(catkin REQUIRED COMPONENTS)

catkin_package()

cmake_minimum_required(VERSION 2.8.3)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

message(STATUS "CATKIN PACKAGE PLACE: ${CATKIN_PACKAGE_SHARE_DESTINATION}")

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  param
  scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
