cmake_minimum_required(VERSION 2.8.3)
project(rrt_path_planner)
set (CMAKE_CXX_STANDARD 11)
## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(octomap REQUIRED)
find_package(fcl REQUIRED)
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  octomap_msgs
  octomap_ros
  roscpp
  std_msgs
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES path_planning
  CATKIN_DEPENDS octomap_msgs octomap_ros roscpp std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
  ${fcl_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

link_libraries(${OCTOMAP_LIBRARIES} ${fcl_LIBRARY_DIRS} ${OMPL_LIBRARY_DIRS})


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(rrt_planner_node src/rrt_planner.cpp src/Planner.cpp)
target_link_libraries(rrt_planner_node
   ${catkin_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${OMPL_LIBRARIES}
   fcl
)

catkin_install_python(PROGRAMS scripts/planner_test.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


catkin_python_setup()
