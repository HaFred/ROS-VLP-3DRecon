cmake_minimum_required(VERSION 3.0.2)
project(image_with_pose)


find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  std_srvs
  message_generation
  tf2
  tf2_ros
  sensor_msgs
  geometry_msgs
  nav_msgs
  turtlebot3_applications_msgs
  # cmake_modules
)

find_package(OpenCV)
# robot spin
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

# catkin_package()
# robot spin
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs sensor_msgs geometry_msgs nav_msgs turtlebot3_applications_msgs cv_bridge image_transport cmake_modules
  DEPENDS Boost Eigen3 OpenCV
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  # robot spin below
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
 lib
 ${catkin_LIB_DIRS} 
)

add_executable(image_with_pose src/image_with_pose_v5.cpp)
# add_executable(${PROJECT_NAME} src/image_with_pose.cpp)
# add_dependencies(turtlebot3_panorama ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_with_pose ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES})

# add_executable(sub_to_rs src/sub_to_rs.cpp)
# target_link_libraries(sub_to_rs ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES})

# add_executable(robot_spinning src/robot_spinning.cpp)
# target_link_libraries(robot_spinning ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES})

#add_executable(image_with_pose src/image_with_pose.cpp)
#target_link_libraries(image_with_pose ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
