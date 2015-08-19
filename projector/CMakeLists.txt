cmake_minimum_required(VERSION 2.8.3)
project(projector)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	libvectormap
	roscpp
	tf
	image_transport
	cv_bridge
	camera_calibration_parsers
)

## System dependencies are found with CMake's conventions
find_package (OpenGL REQUIRED)
find_package (GLUT REQUIRED)
find_package (GLEW REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (Boost REQUIRED)
find_package (OpenCV REQUIRED)
find_package (Qt4 REQUIRED COMPONENTS QtCore QtGui QtOpenGL QtXml)

include (${QT_USE_FILE})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


set (CMAKE_BUILD_TYPE Debug)
set_property (GLOBAL 
        PROPERTY COMPILE_DEFINITIONS DEBUG=1
)
set (CMAKE_AUTOMOC ON)
set (CMAKE_INCLUDE_CURRENT_DIR ON)


################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES projector
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
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
)

## Declare a cpp library
# add_library(projector
#   src/${PROJECT_NAME}/projector.cpp
# )

## Declare a cpp executable
add_executable(
	projector_node 
		projector_node.cpp
		Camera.cpp
		SceneManager.cpp
		PointListDrawObject.cpp
		ShaderProgram.cpp
		SignalsDrawObject.cpp
		LineListDrawObject.cpp
		CameraImageBackground.cpp
		BitmapBackground.cpp
		LaneListDrawObject.cpp
		Framebuffer.cpp
		AreaListDrawObject.cpp
)


add_executable(
	projector_match 
		projector_match.cpp
		Camera.cpp
		SceneManager.cpp
		PointListDrawObject.cpp
		ShaderProgram.cpp
		SignalsDrawObject.cpp
		LineListDrawObject.cpp
		CameraImageBackground.cpp
		BitmapBackground.cpp
		LaneListDrawObject.cpp
		Framebuffer.cpp
		AreaListDrawObject.cpp
		ImageMatcher.cpp
)


add_executable (
	rectangle
		rectangle.cpp
		Camera.cpp
		SceneManager.cpp
		PointListDrawObject.cpp
		ShaderProgram.cpp
		Framebuffer.cpp
		BitmapBackground.cpp
)


add_executable (
	video
		video.cpp
		Camera.cpp
		SceneManager.cpp
		ShaderProgram.cpp
		Framebuffer.cpp
		BitmapBackground.cpp
		CameraImageBackground.cpp
)


add_executable (
	msgdump
		msgdump.cpp
)


add_executable (
	imgtrans
		imgtrans.cpp
)


add_executable (
	vmmatch
		vector_map_match.cpp
		VectorMapMatchWindow.cpp
		VectorMapRenderWidget.cpp
		Camera.cpp
		VectorMapObjects.cpp
)




## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(projector_node projector_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(
	projector_node
		${catkin_LIBRARIES}
		${OPENGL_LIBRARIES}
		${GLUT_LIBRARY}
		${GLEW_LIBRARY}
		${libvectormap_LIBRARIES}
)


target_link_libraries(
	projector_match
		${catkin_LIBRARIES}
		${OPENGL_LIBRARIES}
		${GLUT_LIBRARY}
		${GLEW_LIBRARY}
		${libvectormap_LIBRARIES}
		${OpenCV_LIBRARIES}
)


target_link_libraries (
	rectangle
		${OPENGL_LIBRARIES}
		${GLUT_LIBRARY}
		${GLEW_LIBRARY}
		boost_system
		${OpenCV_LIBRARIES}
)


target_link_libraries (
	video
		${OPENGL_LIBRARIES}
		${GLUT_LIBRARY}
		${GLEW_LIBRARY}
		boost_system
		opencv_imgproc
		${catkin_LIBRARIES}		
)


target_link_libraries (
	msgdump
		${catkin_LIBRARIES}
		sqlite3
)


target_link_libraries (
	imgtrans
		${catkin_LIBRARIES}
)


target_link_libraries (
	vmmatch
		${catkin_LIBRARIES}
		${QT_LIBRARIES}
		${OPENGL_LIBRARIES}
		${GLEW_LIBRARY}
		QGLViewer
		GLU
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS projector projector_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_projector.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
