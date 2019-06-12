cmake_minimum_required(VERSION 2.8.3)
project(g2oodomsave)
add_compile_options(-std=c++11 -Wall -g -fsanitize=leak)
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O2" )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON)
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

add_definitions(-std=c++14)

FIND_PACKAGE(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#maybe important needed in optimzer
FIND_PACKAGE( CSparse )

find_package( catkin REQUIRED )
find_package( G2O REQUIRED )
find_package( Eigen3 REQUIRED)
find_package( Cholmod REQUIRED )
find_package( Sophus REQUIRED )
#find_package(Ceres REQUIRED)
#find_package(glog REQUIRED)
#find_package(GFLAGS REQUIRED)

include_directories( ${Sophus_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
#        ${CERES_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS} ${GFLAGS_INCLUDE_DIR}
        )

include_directories( ${CHOLMOD_INCLUDE_DIR} )
include_directories( ${G2O_INCLUDE_DIRS}
        include/g2oodomsave
        include
        ${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})
##   * add "message_generation" and every package in MSG_DEP_SET to
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  tf_conversions
)
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(g2oSaver
        src/g2oSaver.cpp
        include/g2oSaver.h
        )

add_dependencies(g2oSaver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(g2oSaver
        ${catkin_LIBRARIES}
        g2o_core
        g2o_stuff
        g2o_types_slam3d
        ${PCL_LIBRARIES}
        ${CSPARSE_LIBRARY}
        g2o_csparse_extension
        g2o_types_slam3d
)

add_executable(ndtCalc
        src/ndtTransCalc.cpp
        include/ndtTransCalc.h)

add_dependencies(ndtCalc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ndtCalc
        ${catkin_LIBRARIES}
        g2o_core
        g2o_stuff
        g2o_types_slam3d
        ${PCL_LIBRARIES}
        )

add_executable(lmoptimizer
        include/LMoptimizer.h
        src/LMoptimizer.cpp)

add_dependencies(lmoptimizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lmoptimizer
        ${catkin_LIBRARIES}
        g2o_core g2o_stuff
        g2o_types_slam3d
        g2o_types_slam2d
        ${PCL_LIBRARIES}
        ${CHOLMOD_LIBRARIES}
        g2o_csparse_extension
     #   pthread
        ${CSPARSE_LIBRARY}
       )


add_executable(transfer
        src/odomTransfer.cpp
        )

add_dependencies(transfer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(transfer
        ${catkin_LIBRARIES}
        g2o_core
        g2o_stuff
        g2o_types_slam3d
        ${PCL_LIBRARIES}
        )
add_executable(findPCD
        src/pcd_finder_node.cpp
        src/pcd_finder.cpp
        )

add_dependencies(findPCD ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(findPCD
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(mapping
        include/mapping.h
        src/mapping.cpp
        src/mapping_node.cpp)

add_dependencies(mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(mapping
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${CHOLMOD_LIBRARIES}
        g2o_core g2o_stuff
        g2o_types_slam3d
        g2o_types_slam2d
        g2o_csparse_extension
        ${CSPARSE_LIBRARY}
        )
#air presure
add_executable(airPressureNode
        src/airPresurePub.cpp)
add_dependencies(airPressureNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(airPressureNode
        ${catkin_LIBRARIES}
        )
#solver_csparse types_slam2d  g2o_core g2o_stuff    g2o_types_slam3d g2o_solver_csparse

