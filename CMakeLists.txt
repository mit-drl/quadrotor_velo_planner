cmake_minimum_required(VERSION 2.8.3)
project(quad_ompl)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
)

find_package(OMPL)

catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES quad_ompl
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(quad_ompl_node src/test_ompl.cpp)
target_link_libraries(quad_ompl_node ${OMPL_LIBRARIES})
