cmake_minimum_required(VERSION 2.8.3)
project(pips)

#LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


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
  message_generation
  geometry_msgs
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/HallucinatedRobotModel.cfg
  )

add_service_files(
  FILES
  GenerateDepthImage.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)


catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES collision_checker robot_models
  CATKIN_DEPENDS image_geometry cv_bridge message_runtime
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

add_library(robot_models 
  src/rectangular_model.cpp
  src/cylindrical_model.cpp
  )

add_dependencies(robot_models
   ${PROJECT_NAME}_gencfg
   )

## Declare a cpp executable
#add_executable(trajectory_tester src/test_trajectory.cpp )


target_link_libraries(collision_checker
  robot_models
  ${catkin_LIBRARIES}


 )

 
#target_link_libraries(trajectory_tester
#  ${catkin_LIBRARIES}
#  collision_checker
# )
