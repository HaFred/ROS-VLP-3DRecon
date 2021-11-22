cmake_minimum_required(VERSION 3.0.2)
project(vlc_positioning)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs 
  roscpp tf)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME} src/single_LEDpositioning.cpp)
add_executable(conversion_node src/conversion_node.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


target_link_libraries(conversion_node
  ${catkin_LIBRARIES}
)