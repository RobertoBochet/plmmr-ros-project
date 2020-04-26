cmake_minimum_required(VERSION 2.8.3)
project(raw_data_handler)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf sensor_msgs message_filters)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(raw_data_handler src/raw_data_handler.cpp)
target_link_libraries(raw_data_handler ${catkin_LIBRARIES})

