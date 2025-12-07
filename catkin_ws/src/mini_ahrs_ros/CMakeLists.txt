cmake_minimum_required(VERSION 3.0.2)
project(mini_ahrs_ros VERSION 0.0.0 LANGUAGES CXX)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    tf2
    nodelet
    tf2_ros
    tf2_geometry_msgs
    serial
)

catkin_package(
  INCLUDE_DIRS include modules/mini_ahrs_driver/include
  LIBRARIES mini_ahrs_ros mini_ahrs_driver
  CATKIN_DEPENDS roscpp nodelet tf2 tf2_ros tf2_geometry_msgs serial
  # DEPENDS system_lib
)

add_subdirectory(modules/mini_ahrs_driver)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  "modules/mini_ahrs_driver/include"
)

add_library(${PROJECT_NAME}
    src/MiniAHRSNodelet.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    mini_ahrs_driver
)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS mini_ahrs_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(FILES mini_ahrs_ros.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
