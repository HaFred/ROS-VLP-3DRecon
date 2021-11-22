cmake_minimum_required(VERSION 2.8.3)
project(imu_tools)

find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs std_msgs tf)

catkin_package()

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11")
endif()

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/imu_compass.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS scripts/compute_calibration
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
