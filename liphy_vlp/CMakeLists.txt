cmake_minimum_required(VERSION 3.0.2)
project(liphy_vlp)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(OpenCV)

add_message_files(
  FILES
  liphylight.msg
)


generate_messages(
   DEPENDENCIES
   std_msgs
)


catkin_package(

CATKIN_DEPENDS message_runtime

)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
 lib
 ${catkin_LIB_DIRS} 
)

find_package(PkgConfig)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)
include_directories(${YAMLCPP_INCLUDE_DIRS})

add_executable(liphy_vlp src/liphy_vlp.cpp)
target_link_libraries(liphy_vlp liblightfly.so ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAMLCPP_LIBRARIES})

add_executable(liphy_light_selection src/liphy_light_selection.cpp)
target_link_libraries(liphy_light_selection liblightfly.so ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAMLCPP_LIBRARIES})

