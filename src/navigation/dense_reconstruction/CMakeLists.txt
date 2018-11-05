cmake_minimum_required(VERSION 2.8.3)
project(dense_reconstruction)

#set(CMAKE_CXX_FLAGS "-std=c++11")
list(APPEND CMAKE_MODULE_PATH  ${PROJECT_SOURCE_DIR}/cmake)

set(CMAKE_BUILD_TYPE Release)
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_conversions
  pcl_ros
  geometry_msgs
  image_transport
  tf
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES stereo_dense_reconstruction
  #CATKIN_DEPENDS cv_bridge dynamic_reconfigure geometry_msgs image_transport roscpp rospy sensor_msgs std_msgs pcl_conversions pcl_ros
  #DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


set(LIBELAS_SRC_DIR src/elas)
set(CMAKE_CXX_FLAGS "-msse3")
set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 

find_package(OpenCV REQUIRED)
find_package(Boost 1.54.0 COMPONENTS system filesystem program_options REQUIRED)
find_package(Eigen3 3.0 REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${LIBELAS_SRC_DIR}
)

FILE(GLOB LIBELAS_SRC_FILES "src/elas/*.cpp")

add_executable(dense_reconstruction_node ${LIBELAS_SRC_FILES} src/dense_reconstruction.cpp)
target_link_libraries(dense_reconstruction_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
