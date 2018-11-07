CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
PROJECT(frontier_exploration)
add_definitions(-std=c++11 -g -O3)

find_package(catkin REQUIRED COMPONENTS
		roscpp
		nav_msgs
		geometry_msgs
    message_generation
		)
find_package(OpenCV REQUIRED)

add_service_files(
  FILES
  GetFrontiers.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package(
        INCLUDE_DIRS 
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS nav_msgs geometry_msgs roscpp message_generation
        DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
)

add_executable(frontier_exploration_node src/frontier_exploration_node.cpp)

target_link_libraries(frontier_exploration_node
		${catkin_LIBRARIES}
		${OpenCV_LIBRARIES}
		)

add_library(frontier_exploration src/frontier_exploration_node.cpp)

target_link_libraries(frontier_exploration
		${catkin_LIBRARIES}
		${OpenCV_LIBRARIES}
		)