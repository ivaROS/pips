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
  tf2
  tf2_eigen
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES collision_checker
  CATKIN_DEPENDS image_geometry
  DEPENDS Eigen3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
  include
)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

## Declare the collision_checker library
add_library(collision_checker src/collision_checker.cpp)

## Declare a cpp executable
#add_executable(draw_frames src/draw_frames.cpp)
#add_executable(trajectory_tester src/test_trajectory.cpp )


target_link_libraries(collision_checker
  ${catkin_LIBRARIES}
 )

#target_link_libraries(draw_frames
#  ${catkin_LIBRARIES}
# )
 
 
#target_link_libraries(trajectory_tester
#  ${catkin_LIBRARIES}
#  collision_checker
# )
