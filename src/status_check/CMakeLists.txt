cmake_minimum_required(VERSION 2.8.3)
project(status_check)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs dynamic_reconfigure)

generate_dynamic_reconfigure_options(
		cfg/parameters.cfg
)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(status_check src/status_check.cpp)
add_dependencies(status_check ${PROJECT_NAME}_gencfg)
target_link_libraries(status_check ${catkin_LIBRARIES})
