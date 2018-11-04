CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
PROJECT(exploration)
add_definitions(-std=c++11 -g -O3)

find_package(catkin REQUIRED COMPONENTS
		roscpp
		nav_msgs
		geometry_msgs
		)
find_package(OpenCV REQUIRED)

catkin_package(
        INCLUDE_DIRS 
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS nav_msgs geometry_msgs roscpp 
        DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
)

add_executable(exploration_node src/exploration_node.cpp)

target_link_libraries(exploration_node
		${catkin_LIBRARIES}
		${OpenCV_LIBRARIES}
		)