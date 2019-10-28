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
#  image_view  #Any package intended to run on the robot should not require any gui packages
  tf
  tf2
  tf2_eigen
  tf2_geometry_msgs
  message_generation
  geometry_msgs
  dynamic_reconfigure
   pcl_ros
   urdf
   visualization_msgs #just for debugging
#  dense_depth_projection
#  extended_local
)

find_package(OpenMP)

#tf2_eigen must be at least version 0.5.15

#find_package(Eigen3 REQUIRED)

#Will work with either 2.4 or 3.0+
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/HallucinatedRobotModel.cfg
  cfg/RobotModel.cfg
  )

add_service_files(
  FILES
  GenerateDepthImage.srv
  TestCollision.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include ${OpenCV_INCLUDE_DIRS} #${EIGEN3_INCLUDE_DIR} 
  LIBRARIES collision_checker pips_collision_checker depth_image_collision_checker
  CATKIN_DEPENDS image_geometry cv_bridge message_runtime urdf geometry_msgs#extended_local dense_depth_projection
  DEPENDS  OpenCV tf2_eigen #Eigen3
)

include_directories(
  include

  ${catkin_INCLUDE_DIRS}
#  ${EIGEN3_INCLUDE_DIR}
#  ${PROJECT_SOURCE_DIR}
  ${OpenCV_INCLUDE_DIRS}

#  include
)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -fopenmp")	#TODO: remove these and at least switch to add_definitions, though openmp should really only be enabled on target that needs it


# add_library(robot_model_interface src/hallucinated_robot_model_interface.cpp)
# 
## Declare the collision_checker library
add_library(collision_checker src/collision_checker.cpp)
# 
add_library(pips_collision_checker src/pips_collision_checker.cpp)
# 
add_library(depth_image_collision_checker src/depth_image_collision_checker.cpp)


# 
# #message([WARN] "project Exported: ${${PROJECT_NAME}_EXPORTED_TARGETS}\n catkin exported: ${catkin_EXPORTED_TARGETS} \n gencfg: ${PROJECT_NAME}_gencfg")
# 
# ##${${PROJECT_NAME}_EXPORTED_TARGETS} is necessary to ensure that project messages get built before library
# ##${catkin_EXPORTED_TARGETS} ensures that all messages for all packages on which this package depends (via find_package(catkin...)
# ###${PROJECT_NAME}_gencfg used to be used to ensure reconfiguration generation happened first, but that is actually included in ${${PROJECT_NAME}_EXPORTED_TARGETS}
# add_dependencies(robot_model_interface
#    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} #${PROJECT_NAME}_gencfg 
#    )

add_dependencies(collision_checker
   ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
   )
#    
add_dependencies(pips_collision_checker
   ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
   )
   
add_dependencies(depth_image_collision_checker
   ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
   )


target_link_libraries(collision_checker
  ${catkin_LIBRARIES}
  )

target_link_libraries(pips_collision_checker
  collision_checker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

target_link_libraries(depth_image_collision_checker
  pips_collision_checker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )
 

 

if(OpenCV_VERSION VERSION_LESS "3.0")
  MESSAGE(STATUS "Your OpenCV version is less than 3.0, OpenCL benchmarker will not be built")
else()
  add_executable(ocl_benchmarker src/approach_benchmarker.cpp)
  target_link_libraries(ocl_benchmarker
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )
endif()


  
add_executable(omp_benchmarker src/omp_benchmarker.cpp)
target_link_libraries(omp_benchmarker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

target_compile_options(omp_benchmarker PRIVATE -g  -save-temps -fverbose-asm   -march=native ${OpenMP_CXX_FLAGS} -ftree-vectorize -fassociative-math -fno-signed-zeros -fno-trapping-math  -ffast-math ) # -fopt-info-vec-all  -freciprocal-math #-funsafe-math-optimizations #-fno-math-errno -ffinite-math-only -fno-rounding-math -fno-signaling-nans -fcx-limited-range -fexcess-precision=fast
