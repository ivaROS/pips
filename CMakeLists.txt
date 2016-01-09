cmake_minimum_required(VERSION 2.8.3)
project(pips)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_geometry
  image_transport
  image_view
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native -std=c++11")


## Declare a cpp executable
add_executable(draw_frames src/draw_frames.cpp)

target_link_libraries(draw_frames
  ${catkin_LIBRARIES}
 )

