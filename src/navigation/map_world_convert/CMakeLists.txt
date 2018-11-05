cmake_minimum_required(VERSION 2.8.3)
project(map_world_convert)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  geometry_msgs
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES simple_line_planner
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)

include_directories(include
${catkin_INCLUDE_DIRS}
)

add_executable(map_world_convert src/map_world_convert.cpp)
target_link_libraries(map_world_convert ${catkin_LIBRARIES})

install(TARGETS map_world_convert 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
