# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(localiser)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug)
ENDIF()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

#set(CMAKE_CXX_FLAGS_RELEASE "-O2")

find_package(MRPT REQUIRED base gui obs slam)
 
if (MRPT_VERSION VERSION_LESS 2.0.0)
  set(CMAKE_CXX_FLAGS "-Wno-deprecated-declarations ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  gps_common
  custom_point_types
  dataset_tools
  h264_bag_playback
  cv_bridge
  tf
  std_msgs
  nav_msgs
  sensor_msgs
  mrpt_bridge
  roslaunch
  rosbag
  tf2
  tf2_geometry_msgs
  localiser_core
)

#find mrpt packages
#find_package(MRPT REQUIRED base gui obs slam)

#if (MRPT_VERSION VERSION_LESS 2.0.0)
#        set(CMAKE_CXX_FLAGS "-Wno-deprecated-declarations ${CMAKE_CXX_FLAGS}")
#endif()

find_package(OpenCV 3 REQUIRED)

find_package(Eigen3 REQUIRED)
#find_package(GTSAM REQUIRED)

# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse, QGLViewer, and Eigen3.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)
find_package(CSparse)
find_package(G2O REQUIRED)

find_package(Boost COMPONENTS system filesystem REQUIRED)


find_package(PCL REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Generate services in the 'srv' folder
add_service_files(
        FILES
        instruct_localiser.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS nav_msgs sensor_msgs std_msgs sensor_msgs mrpt_bridge rosbag localiser_core

  INCLUDE_DIRS include
)

###########
## Build ##
###########

# eigen3 is header only
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR}
  ${G2O_INCLUDE_DIRS}
  ${MRPT_LIBS}
#  gtsam
  include/${PROJECT_NAME}
  include
  ${PCL_INCLUDE_DIRS}
)

# collect all g2o libs
set(g2o_LIB_collection
    g2o_core
    #g2o_types_slam3d
    g2o_stuff
    #g2o_types_slam3d_addons
    #g2o_hierarchical
    #g2o_simulator
    #g2o_types_data
    g2o_types_slam2d
    #g2o_types_sba
    #g2o_types_sim3
    #g2o_types_icp
    #g2o_types_sclam2d g2o_types_slam2d_addons
    #g2o_solver_pcg g2o_solver_dense g2o_solver_structure_only
    g2o_solver_csparse g2o_csparse_extension #g2o_solver_slam2d_linear
    #g2o_solver_cholmod g2o_solver_eigen
)

#add_library(icp_map
#    src/icp_map_maker.cpp
#)
#add_library(simple_graph
#    src/simple_graph.cpp
#)

add_executable(${PROJECT_NAME}
    src/ros_localiser.cpp
    src/bag_input.cpp
  #  src/localiser_input.cpp
  #  src/localiser_output.cpp
  #  src/observer.cpp
  #  src/predictor.cpp
  #  src/localisation_method.cpp
  #  src/linear_filter.cpp
  #  src/graph_optimiser.cpp
  #  src/gtsam_optimiser.cpp
  #  src/publisher.cpp
  #  src/icp_matcher_pipeline.cpp
    #src/icp_map_maker.cpp
)
#add_executable(edit_g2o
#    src/edit_g2o.cpp
#    #src/icp_map_maker.cpp
#)
#add_executable(rewrite_bag
#    src/rewrite_bag.cpp
#    #src/icp_map_maker.cpp
#)

#add_executable(dropout_features
#    src/dropout_features.cpp
#    #src/icp_map_maker.cpp
#)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
#    icp_map
#    simple_graph
 ${MRPT_LIBS} 
# ${PCL_LIBRARIES}
    ${CSPARSE_LIBRARY} # csparse is used for graph optimization
#    ${g2o_LIB_collection}
 ${G2O_CORE_LIBRARY}
 ${G2O_SOLVER_SLAM2D_LINEAR}
 ${G2O_STUFF_LIBRARY}
 ${G2O_TYPES_SLAM2D}
 ${G2O_SOLVER_CHOLMOD}
 ${G2O_SOLVER_CSPARSE}
 ${G2O_SOLVER_CSPARSE_EXTENSION}
# gtsam
# ${GTSAM_LIBS}
  ${Boost_SYSTEM_LIBRARY}

)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  #launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )

#############
## Testing ##
#############
# Add testing for *.launch files
#roslaunch_add_file_check(launch)
