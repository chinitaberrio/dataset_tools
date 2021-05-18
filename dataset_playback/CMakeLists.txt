cmake_minimum_required(VERSION 2.8.3)
project(dataset_playback)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(DISTRO $ENV{ROS_DISTRO})
message(STATUS "Distribution: " ${DISTRO})

#set(DEFAULT_BUILD_QT5 OFF)
#if(${DISTRO} STREQUAL "kinetic")
#    set(DEFAULT_BUILD_QT5 ON)
#endif()
set(DEFAULT_BUILD_QT5 ON)

# package currently needs opencv
find_package(OpenCV 3 REQUIRED)

#option(UseQt5 "Build w/ QT version 5" ${DEFAULT_BUILD_QT5})
set(UseQt5 True)

if (UseQt5)
	#find_package(Qt5 COMPONENTS Core Gui Network Concurrent REQUIRED)
	find_package(Qt5Widgets)
	find_package(Qt5Core)
	find_package(Qt5Gui)
        find_package(Qt5Svg)
        find_package(Qt5Xml)
else()
        find_package(Qt4 COMPONENTS QtCore QtGui QtNetwork QtSvg REQUIRED)
	include(${QT_USE_FILE})
endif()

#find_package(OpenCV REQUIRED)
#find_package(PkgConfig REQUIRED)

#pkg_check_modules(OGRE_OV OGRE OGRE-Overlay)

# Old versions of OGRE (pre 1.9) included OGRE-Overlay in the main package
# (i.e. there was no OGRE-Overlay component).  So if the above
# pkg_check_modules() failed, try looking for just OGRE.
#if(NOT OGRE_OV_FOUND)
#  pkg_check_modules(OGRE_OV REQUIRED OGRE)
#endif()

find_package(graphviz REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  h264_bag_playback
  dataset_tools
  py_trees_msgs
  sensor_msgs
  zio_navigation_msgs
)

#find_package(graphviz)

catkin_package(CATKIN_DEPENDS nav_msgs sensor_msgs)

set(${PROJECT_NAME}_SOURCES
  src/bus_stop_panel.cpp
  src/QBusStopScene.cpp
  src/main.cpp
)

set(${PROJECT_NAME}_HEADERS
  include/${PROJECT_NAME}/bus_stop_panel.h
  include/${PROJECT_NAME}/QBusStopScene.h
)

# invoke MOC and UI/ include Qt headers/ link Qt libraries
if (UseQt5)
	qt5_wrap_cpp(${PROJECT_NAME}_MOCSrcs ${${PROJECT_NAME}_HEADERS})
	include_directories(
		${Qt5Widgets_INCLUDE_DIRS}
		${Qt5Core_INCLUDE_DIRS}
		${Qt5Gui_INCLUDE_DIRS}
                ${Qt5Xml_INCLUDE_DIRS}
                ${Qt5Svg_INCLUDE_DIRS}
                ${Qt5Network_INCLUDE_DIRS}
		${Qt5Concurrent_INCLUDE_DIRS}
		)
	link_libraries(
		${Qt5Core_LIBRARIES}
		${Qt5Widgets_LIBRARIES}
		${Qt5Gui_LIBRARIES}
                ${Qt5Xml_LIBRARIES}
                ${Qt5Svg_LIBRARIES}
                ${Qt5Network_LIBRARIES}
		${Qt5Concurrent_LIBRARIES}
		)
else()
	qt4_wrap_cpp(${PROJECT_NAME}_MOCSrcs ${${PROJECT_NAME}_HEADERS})
	include_directories(${Qt4_INCLUDE_DIR})
	link_libraries(
  		${QT_LIBRARIES}
		)
endif()

# Other includes
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
  ${QT_LIBRARIES}
  ${graphviz_INCLUDE_DIRECTORIES}
  include/${PROJECT_NAME}
)

#LINK_DIRECTORIES( ${LINK_DIRECTORIES} )

# Other libraries
link_libraries(
  ${QT_LIBRARIES}
  ${graphviz_GVC_LIBRARY}
  ${catkin_LIBRARIES}

${OpenCV_LIBRARIES}
  cgraph
)

add_definitions("-Wall -Wunused -std=c++11")

set(PROJECT_SOURCE_FILES
  ${${PROJECT_NAME}_SOURCES}
  ${${PROJECT_NAME}_MOCSrcs}
)

add_executable(${PROJECT_NAME}
  ${PROJECT_SOURCE_FILES}
)

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


install(DIRECTORY launch
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY destination_files
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
