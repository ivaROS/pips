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
  message_generation
  geometry_msgs
  dynamic_reconfigure
   pcl_ros
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
  LIBRARIES collision_checker pips_collision_checker
  CATKIN_DEPENDS image_geometry cv_bridge message_runtime #extended_local dense_depth_projection
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

## Declare the collision_checker library
add_library(collision_checker src/collision_checker.cpp)

add_library(pips_collision_checker src/pips_collision_checker.cpp)

add_library(robot_model_interface src/hallucinated_robot_model_interface.cpp)


#message([WARN] "project Exported: ${${PROJECT_NAME}_EXPORTED_TARGETS}\n catkin exported: ${catkin_EXPORTED_TARGETS} \n gencfg: ${PROJECT_NAME}_gencfg")

##${${PROJECT_NAME}_EXPORTED_TARGETS} is necessary to ensure that project messages get built before library
##${catkin_EXPORTED_TARGETS} ensures that all messages for all packages on which this package depends (via find_package(catkin...)
###${PROJECT_NAME}_gencfg used to be used to ensure reconfiguration generation happened first, but that is actually included in ${${PROJECT_NAME}_EXPORTED_TARGETS}
add_dependencies(robot_model_interface
   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} #${PROJECT_NAME}_gencfg 
   )

add_dependencies(collision_checker
   ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
   )
   
add_dependencies(pips_collision_checker
   ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
   )

#add_library (dense_model 
#  src/dense_model.cpp
#  )

#SET(ROBOT_MODEL_LIBS dense_model)

  
add_library(rectangular_model
  src/rectangular_model.cpp
  )
target_compile_options(rectangular_model PRIVATE -march=native -ffast-math -ftree-vectorize ) #-ftree-vectorizer-verbose=7
target_compile_options(rectangular_model PUBLIC  -std=c++11)

target_link_libraries(rectangular_model
  ${catkin_LIBRARIES}
  )
SET(ROBOT_MODEL_LIBS ${ROBOT_MODEL_LIBS} rectangular_model)

add_library(rectangular_model_ss
  src/rectangular_model_ss.cpp
  )
  
#For verbose vectorizing output on gcc 4.6: -ftree-vectorizer-verbose=7
#For verbose vectorizing output on gcc 5.4: -fopt-info-vec-all
target_compile_options(rectangular_model_ss PRIVATE -march=native ${OpenMP_CXX_FLAGS} -ftree-vectorize -fassociative-math -fno-signed-zeros -fno-trapping-math  -ffast-math  ) # -freciprocal-math #-funsafe-math-optimizations #-fno-math-errno -ffinite-math-only -fno-rounding-math -fno-signaling-nans -fcx-limited-range -fexcess-precision=fast
target_compile_options(rectangular_model_ss PUBLIC  -std=c++11)
target_link_libraries(rectangular_model_ss
  ${catkin_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  )
SET(ROBOT_MODEL_LIBS ${ROBOT_MODEL_LIBS} rectangular_model_ss)

add_library(cylindrical_model_t_vect
  src/cylindrical_model_t_vect.cpp
  )
target_compile_options(cylindrical_model_t_vect PRIVATE -march=native -ffast-math -ftree-vectorize ) #-ftree-vectorizer-verbose=7
target_compile_options(cylindrical_model_t_vect PUBLIC  -std=c++11)
target_link_libraries(cylindrical_model_t_vect
  ${catkin_LIBRARIES}
  )
SET(ROBOT_MODEL_LIBS ${ROBOT_MODEL_LIBS} cylindrical_model_t_vect)

if(OpenCV_VERSION VERSION_LESS "3.0")
  MESSAGE(STATUS "Your OpenCV version is less than 3.0, OpenCL model will not be built")
else()
  add_library(rectangular_model_ocl
    src/rectangular_model_ocl.cpp
    )
  target_compile_options(rectangular_model_ocl PRIVATE -g)
  target_link_libraries(rectangular_model_ocl
    ${catkin_LIBRARIES}
    -g
    )
  SET(ROBOT_MODEL_LIBS ${ROBOT_MODEL_LIBS} rectangular_model_ocl)
endif()

add_library(rectangular_model_pf
  src/rectangular_model_pf.cpp
  )
target_link_libraries(rectangular_model_pf
  ${catkin_LIBRARIES}
  )
SET(ROBOT_MODEL_LIBS ${ROBOT_MODEL_LIBS} rectangular_model_pf)

add_library(cylindrical_model 
  src/cylindrical_model.cpp
  )
#target_compile_options(cylindrical_model PRIVATE -ftree-vectorize -ftree-vectorizer-verbose=7)
#target_compile_options(cylindrical_model PUBLIC  -std=c++11)
target_link_libraries(cylindrical_model
  ${catkin_LIBRARIES}
  )
SET(ROBOT_MODEL_LIBS ${ROBOT_MODEL_LIBS} cylindrical_model)

add_library(cylindrical_model_t 
  src/cylindrical_model_t.cpp
  )
target_link_libraries(cylindrical_model_t
  ${catkin_LIBRARIES}
  )
SET(ROBOT_MODEL_LIBS ${ROBOT_MODEL_LIBS} cylindrical_model_t)

MESSAGE(STATUS "ROBOT MODELS " ${ROBOT_MODEL_LIBS})
   
target_link_libraries(robot_model_interface
  ${ROBOT_MODEL_LIBS}
  ${catkin_LIBRARIES}
  )

target_link_libraries(collision_checker
  ${catkin_LIBRARIES}
  )

target_link_libraries(pips_collision_checker
  collision_checker
  robot_model_interface
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

