cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_velo_planner)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    std_msgs
    octomap_msgs
    pcl_ros
    pcl_conversions
    pcl_msgs
    sensor_msgs
)

# find_package(OMPL REQUIRED)
find_package(octomap REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp pcl_ros pcl_conversions pcl_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  include
)

link_directories(/usr/local/lib)

link_libraries(${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ompl)

add_executable(test_ompl src/test_ompl.cpp)
add_executable(quad_ompl_node
    src/quad_ompl_node.cpp
    src/collision_checker.cpp
    src/clustering.cpp)
