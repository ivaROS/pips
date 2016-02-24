cmake_minimum_required(VERSION 2.8.3)
project(pips)


# don't use opencv as found by any prior running of find_package
unset(OpenCV_CONFIG_PATH CACHE)
unset(OpenCV_DIR CACHE)

set(TMP_PREFIX_PATH ${CMAKE_PREFIX_PATH})
set(CMAKE_PREFIX_PATH "/opt/opencv")
find_package(OpenCV 3.1 REQUIRED)

# restore CMAKE_PREFIX_PATH and other cached variables
# so nothing other package finds this opencv
set(CMAKE_PREFIX_PATH ${TMP_PREFIX_PATH})

unset(OpenCV_CONFIG_PATH CACHE)
unset(OpenCV_DIR CACHE)



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



catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES collision_checker
  #CATKIN_DEPENDS 
  DEPENDS Eigen3 image_geometry
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}

  include
)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

## Declare the collision_checker library
add_library(collision_checker src/collision_checker.cpp)

## Declare a cpp executable
add_executable(draw_frames src/draw_frames.cpp)
add_executable(trajectory_tester src/test_trajectory.cpp )


target_link_libraries(collision_checker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}

 )

target_link_libraries(draw_frames
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}

 )
 
 
target_link_libraries(trajectory_tester
  ${catkin_LIBRARIES}
  collision_checker
 )
