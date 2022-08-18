cmake_minimum_required(VERSION 3.1.0)

project(dataset_gui VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  h264_bag_playback
  dataset_tools
  dataset_msgs
  sensor_msgs
  custom_point_types
  tf
  tf_conversions
  rosgraph_msgs
  tf2_msgs
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

include_directories(
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
    ${PROJECT_SOURCE_DIR}/src
    ${catkin_INCLUDE_DIRS}
)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

# package currently needs opencv
find_package(OpenCV 3 REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(

  CATKIN_DEPENDS h264_bag_playback
    #INCLUDE_DIRS 
)


add_executable(${PROJECT_NAME}
    src/datasetgui.ui
    src/datasetgui.cpp
    src/main.cpp
    src/treemodel.cpp
    src/treeitem.cpp
    # following QT classes are included in the add_executable to allow the qt moc to find them
    include/${PROJECT_NAME}/datasetgui.h
    include/${PROJECT_NAME}/treemodel.h
    include/${PROJECT_NAME}/treeitem.h
    include/${PROJECT_NAME}/dataset_thread.h
    include/${PROJECT_NAME}/datasetgui.h
)

target_link_libraries(${PROJECT_NAME} 
    ${catkin_LIBRARIES}
    Qt5::Widgets
    ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
