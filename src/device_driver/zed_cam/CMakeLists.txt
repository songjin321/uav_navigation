CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
PROJECT(zed_cam)
add_definitions(-std=c++11 -g -O3)

find_package(catkin REQUIRED COMPONENTS
		roscpp
		sensor_msgs
		cv_bridge
		image_transport)
find_package(OpenCV REQUIRED)

catkin_package(
        INCLUDE_DIRS 
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
        DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
)

add_executable(zed_cam_node src/zed_cam_node.cpp)

target_link_libraries(zed_cam_node
		${catkin_LIBRARIES}
		${OpenCV_LIBRARIES}
		)
