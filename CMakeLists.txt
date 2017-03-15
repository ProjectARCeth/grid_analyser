cmake_minimum_required(VERSION 2.8.3)
project(grid_analyser)

find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  catkin_simple
  arc_tools
    )


catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
src/gridAnalyser.cpp
)

add_executable(grid_analyser_node src/gridAnalyser_node.cpp)
target_link_libraries(grid_analyser_node ${catkin_LIBRARIES} ${PROJECT_NAME})

cs_install()
cs_export()
