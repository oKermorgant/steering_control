cmake_minimum_required(VERSION 3.0.2)
project(steering_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
roscpp
nav_msgs
sensor_msgs
tf2_ros
urdf
costmap_2d
dwa_local_planner
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
catkin_package(
INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
CATKIN_DEPENDS
roscpp
nav_msgs
sensor_msgs
tf2_ros
urdf
costmap_2d
dwa_local_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# generic local_planner + local costmap (eg control lib)
add_library(control src/control_nodes/control.cpp)
target_include_directories(control PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(control ${catkin_LIBRARIES} yaml-cpp)

# Example on how to use DWA class / plugin into an independant control node
add_executable(dwa_unicycle src/control_nodes/dwa_unicycle.cpp)
target_include_directories(dwa_unicycle PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(dwa_unicycle control)

# Example of a stupid planner that aims at the goal and does not care about walls
add_executable(greedy_nowalls src/greedy_nowalls_plugin.cpp src/control_nodes/greedy_nowalls.cpp)
target_include_directories(greedy_nowalls PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(greedy_nowalls control)
