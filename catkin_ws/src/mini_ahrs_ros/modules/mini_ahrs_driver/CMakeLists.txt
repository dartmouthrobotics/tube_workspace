cmake_minimum_required(VERSION 3.1)
PROJECT(MiniAHRSDriver VERSION 0.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)

# ROS serial library for interacting with the usb port
# find_package(serial REQUIRED)
find_package (catkin REQUIRED COMPONENTS
    roscpp
    serial
)
find_package(Threads REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

FILE(GLOB all_source_files ./src/*.cpp)
# include_directories(include ${serial_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(mini_ahrs_driver ${all_source_files})
# target_link_libraries(mini_ahrs_driver ${serial_LIBRARIES} Threads::Threads)
target_link_libraries(mini_ahrs_driver ${catkin_LIBRARIES} Threads::Threads)

add_executable(read_from_ahrs examples/read_from_ahrs.cpp)
target_link_libraries(read_from_ahrs mini_ahrs_driver Threads::Threads)
