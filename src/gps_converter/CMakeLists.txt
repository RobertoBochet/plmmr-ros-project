cmake_minimum_required(VERSION 2.8.3)
project(gps_converter)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf sensor_msgs message_filters)

generate_messages(
        DEPENDENCIES
        sensor_msgs
)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(converter src/converter.cpp)
target_link_libraries(converter ${catkin_LIBRARIES})

