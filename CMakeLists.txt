cmake_minimum_required(VERSION 2.8.3)
project(imu_gps_localization)

add_compile_options(-std=c++14)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED PATHS /usr/include/eigen3)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
)

catkin_package()

add_subdirectory(imu_gps_localizer)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/imu_gps_localizer/include
  ${PROJECT_SOURCE_DIR}/ros_wrapper/include
)

add_library(ros_wrapper_lib
	ros_wrapper/src/localization_wrapper.cpp
)
target_link_libraries(ros_wrapper_lib
  ${catkin_LIBRARIES}
  imu_gps_localizer_lib
  glog
)

## Localization node.
add_executable(${PROJECT_NAME}_node ros_wrapper/src/localization_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ros_wrapper_lib
)
