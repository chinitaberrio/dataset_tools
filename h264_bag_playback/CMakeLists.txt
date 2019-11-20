cmake_minimum_required(VERSION 2.8.3)
project(h264_bag_tools)

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
  LIBRARIES h264_bag_tools
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
add_executable(h264_bag_tools_node src/main.cpp src/video.cpp src/helper_functions.cpp)
target_link_libraries(h264_bag_tools_node
		${catkin_LIBRARIES}
		${OpenCV_LIBS})

add_library(h264_bag_tools src/main.cpp src/video.cpp src/helper_functions.cpp)
target_link_libraries(h264_bag_tools
		${catkin_LIBRARIES}
		${OpenCV_LIBS})

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(h264_bag_tools
		  ${catkin_EXPORTED_LIBRARIES}
		  ${catkin_LIBRARIES}
		  ${OpenCV_LIBS})
endif()

install(TARGETS h264_bag_tools_node
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark the nodelet library for installations
install(TARGETS h264_bag_tools
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES nodelet_definition.xml 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
