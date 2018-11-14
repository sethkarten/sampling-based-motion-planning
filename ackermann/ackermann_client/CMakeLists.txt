cmake_minimum_required(VERSION 2.8.3)
project(ackermann_client)


###########
## Build ##
###########
find_package(catkin REQUIRED COMPONENTS roscpp ackermann_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} )

add_executable(navigate navigate.cc)
add_dependencies(navigate ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigate ${catkin_LIBRARIES})
