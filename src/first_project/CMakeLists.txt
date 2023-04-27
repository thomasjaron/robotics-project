cmake_minimum_required(VERSION 2.8.3)
project(first_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

## Custom messages and services
add_message_files( FILES Odom.msg )
add_service_files( FILES reset_odom.srv)
generate_messages( DEPENDENCIES std_msgs )


## Declare a catkin package
catkin_package( CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom_node src/odometry.cpp)
target_link_libraries(odom_node ${catkin_LIBRARIES})

## Insert here the stuff for custom messages and services
add_dependencies(odom_node first_project_generate_messages_cpp)
add_dependencies(odom_node ${catkin_EXPORTED_TARGETS})

#add_executable(sub src/sub.cpp)
#target_link_libraries(sub ${catkin_LIBRARIES})

