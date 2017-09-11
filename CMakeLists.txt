cmake_minimum_required(VERSION 2.8.3)
project(ecn_ros2016)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  ecn_common
)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES ecn_ros2016
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs cv_bridge image_transport ecn_common
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_executable(baxter_color main.cpp)
target_link_libraries(baxter_color ${catkin_LIBRARIES})
