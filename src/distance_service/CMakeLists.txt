cmake_minimum_required(VERSION 2.8.3)
project(distance_service)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs message_generation)

add_service_files(
		FILES
		DistanceService.srv
)

generate_messages(
		DEPENDENCIES
		std_msgs
		geometry_msgs
)


## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(distance src/distance.cpp)
target_link_libraries(distance ${catkin_LIBRARIES})
add_dependencies(distance ${catkin_EXPORTED_TARGETS})