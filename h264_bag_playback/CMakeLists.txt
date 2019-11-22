cmake_minimum_required(VERSION 2.8.3)
project(h264_bag_playback)

find_package(catkin REQUIRED COMPONENTS nodelet
	rosbag
	rosbag_storage
	gmsl_frame_msg
	roscpp
	image_transport
	std_msgs
	cv_bridge
	nav_msgs
	sensor_msgs)

find_package(OpenCV REQUIRED)

## Setup include directories
include_directories(${catkin_INCLUDE_DIRS},
  include/${PROJECT_NAME}
  include)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
	INCLUDE_DIRS include
	CATKIN_DEPENDS nodelet
	roscpp
	rosbag
	rosbag_storage
	gmsl_frame_msg
	image_transport
	std_msgs
	nav_msgs
	sensor_msgs
	cv_bridge
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


## Create the nodelet tutorial library
#add_executable(${PROJECT_NAME}_node
#		src/h264_bag_playback.cpp
#		src/video.cpp
#		src/helper_functions.cpp)

#target_link_libraries(${PROJECT_NAME}_node
#		${catkin_LIBRARIES}
#		${OpenCV_LIBS})
#

add_library(${PROJECT_NAME}
		src/h264_bag_playback.cpp
		src/video.cpp
		src/helper_functions.cpp)

target_link_libraries(${PROJECT_NAME}
		${catkin_LIBRARIES}
		${OpenCV_LIBS})

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME}
		  ${catkin_EXPORTED_LIBRARIES}
		  ${catkin_LIBRARIES}
		  ${OpenCV_LIBS})
endif()
#
#install(TARGETS ${PROJECT_NAME}_node
#		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#

## Mark the nodelet library for installations
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES nodelet_definition.xml 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
