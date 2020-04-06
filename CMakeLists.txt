cmake_minimum_required(VERSION 2.8.3)
project(imu_gps_localization)
# c++14 for make_unique
add_compile_options(-std=c++14)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED PATHS /usr/include/eigen3)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
)

catkin_package()


# 系统包含的所有头文件
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/imu_gps_localizer/include
  ${PROJECT_SOURCE_DIR}/third_party/GeographicLib/include
  ${PROJECT_SOURCE_DIR}/ros_wrapper/include
)


## ROS
file(GLOB_RECURSE SENSORDATA_SRCS "${PROJECT_SOURCE_DIR}/ros_wrapper/src/sensor_data/*.cpp")
file(GLOB_RECURSE SUBSCRIBER_SRCS "${PROJECT_SOURCE_DIR}/ros_wrapper/src/subscriber/*.cpp")
file(GLOB_RECURSE PUBLISHER_SRCS "${PROJECT_SOURCE_DIR}/ros_wrapper/src/publisher/*.cpp")
file(GLOB_RECURSE DATAFLOW_SRCS "${PROJECT_SOURCE_DIR}/ros_wrapper/src/data_flow/*.cpp")
file(GLOB_RECURSE LOCALIZERWRAPPER_SRCS "${PROJECT_SOURCE_DIR}/ros_wrapper/src/localizer/*.cpp")

add_library(ros_wrapper
  ${SENSORDATA_SRCS}
  ${SUBSCRIBER_SRCS}
  ${PUBLISHER_SRCS}
  ${DATAFLOW_SRCS}
  ${LOCALIZERWRAPPER_SRCS}
)

# Add GeographicLib.
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/GeographicLib)

## 定位
file(GLOB_RECURSE LOCALIZER_SRCS "${PROJECT_SOURCE_DIR}/imu_gps_localizer/src/*.cpp")

add_library(imu_gps_localizer
  ${LOCALIZER_SRCS}  
)

target_link_libraries(imu_gps_localizer
  libGeographiccc
)


## 数据预处理节点
add_executable(data_flow_node "${PROJECT_SOURCE_DIR}/ros_wrapper/src/app/data_flow_node.cpp")
target_link_libraries(data_flow_node
  ${catkin_LIBRARIES}
  ros_wrapper
  imu_gps_localizer
  glog
)

## 定位节点
add_executable(localization_node "${PROJECT_SOURCE_DIR}/ros_wrapper/src/app/localization_node.cpp")
target_link_libraries(localization_node
  ${catkin_LIBRARIES}
  ros_wrapper
  imu_gps_localizer
  glog
)