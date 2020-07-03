cmake_minimum_required(VERSION 2.8.3)
project(status_check)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs dynamic_reconfigure nav_msgs message_generation distance_service message_filters)

generate_dynamic_reconfigure_options(
		cfg/parameters.cfg
)

add_message_files(
		FILES
		Status.msg
)

generate_messages(
		DEPENDENCIES
		std_msgs
		nav_msgs
)

catkin_package(
		CATKIN_DEPENDS
		message_runtime
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(status_check src/status_check.cpp)
add_dependencies(status_check ${PROJECT_NAME}_gencfg)
target_link_libraries(status_check ${catkin_LIBRARIES})
