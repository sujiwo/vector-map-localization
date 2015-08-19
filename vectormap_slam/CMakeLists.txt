cmake_minimum_required(VERSION 2.8.3)
project(vectormap_slam)

INCLUDE (FindPkgConfig)

# Activate C++11 Features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


find_package(catkin REQUIRED COMPONENTS
	roscpp
	tf
	image_transport
	cv_bridge
	camera_calibration_parsers
	rospy
	)
find_package (OpenGL REQUIRED)
find_package (GLUT REQUIRED)
find_package (GLEW REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (Boost REQUIRED)
find_package (OpenCV REQUIRED)
find_package (Qt4 REQUIRED COMPONENTS 
	QtCore 
	QtGui 
	QtOpenGL 
	QtXml)
	
pkg_check_modules (GSL REQUIRED gsl)

set (CMAKE_BUILD_TYPE Debug)
set_property (GLOBAL 
        PROPERTY COMPILE_DEFINITIONS DEBUG=1
)
# For Qt 4, to do moc
include (${QT_USE_FILE})
set (CMAKE_AUTOMOC ON)
set (CMAKE_INCLUDE_CURRENT_DIR ON)




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vectormap_slam
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories (
	include
	${EIGEN3_INCLUDE_DIR}
	${OPENGL_INCLUDE_DIR}
	${GLUT_INCLUDE_DIR}
	${SQLITE3_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${BOOST_INCLUDE_DIRS}
	${PNG_INCLUDE_DIRS}
	${roscpp_INCLUDE_DIRS}
	${libvectormap_INCLUDE_DIRS}
	${Qt4_INCLUDE_DIRS}
	${GSL_INCLUDE_DIRS}
)



add_executable (
	vmslam
		main.cpp
		MatchWindow.cpp
		RenderWidget.cpp
		ImageDisplay.cpp	
		DrawObjects.cpp
		Camera.cpp
		vector_map.cpp
		LineList.cpp
		TrafficLights.cpp
		PolesDraw.cpp
		ImageMatch.cpp
		DirectionArrow.cpp
		Pose.cpp
#		Crosswalk.cpp
		AreaList.cpp
		Tesselator.cpp
#		Gutters.cpp
)

target_link_libraries(
	vmslam
		${catkin_LIBRARIES}
		${QT_LIBRARIES}
		${OPENGL_LIBRARIES}
		${GSL_LIBRARIES}
)

catkin_install_python (PROGRAMS imagefilter.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable (
	test_tesselator
		test_tesselator.cpp
		Tesselator.cpp
		vector_map.cpp
)


target_link_libraries (
	test_tesselator
		${OPENGL_LIBRARIES}
		boost_system
)


add_executable (
	test_hist
		test_hist.cpp
		ImageMatch.cpp
		RenderWidget.cpp
		Pose.cpp
		Camera.cpp
)

target_link_libraries (
	test_hist
		${OpenCV_LIBRARIES}
		boost_system
		${QT_LIBRARIES}
		${OPENGL_LIBRARIES}
)