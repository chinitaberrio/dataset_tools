cmake_minimum_required(VERSION 2.8.3)
project(dataset_tools)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")


#find catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  nav_msgs
  sensor_msgs
  rosbag
  image_transport
  tf2
  tf2_geometry_msgs
  gmsl_frame_msg
  custom_point_types
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS nav_msgs sensor_msgs std_msgs sensor_msgs gmsl_frame_msg rosbag
  INCLUDE_DIRS include
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  ${PCL_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
    src/bag_output.cpp
    src/bag_input.cpp
    src/run_pipeline.cpp
    #src/icp_matcher_pipeline.cpp
)


target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)



install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

