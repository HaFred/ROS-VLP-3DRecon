cmake_minimum_required(VERSION 3.0.2)
project(image_with_pose)


find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  message_generation
  tf2
  tf2_ros
)

find_package(OpenCV)

catkin_package()

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
 lib
 ${catkin_LIB_DIRS} 
)

add_executable(${PROJECT_NAME} src/image_with_pose_v3.4.cpp)
# add_executable(${PROJECT_NAME} src/image_with_pose.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(image_with_pose src/image_with_pose.cpp)
#target_link_libraries(image_with_pose ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

