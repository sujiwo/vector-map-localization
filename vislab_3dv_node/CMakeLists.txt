cmake_minimum_required(VERSION 2.8.3)
project(vislab_3dv_node)

find_package (catkin REQUIRED COMPONENTS
	roscpp
	image_transport
	pcl_ros
	pcl_conversions
)

find_package (Boost REQUIRED COMPONENTS 
	system 
	thread 
	chrono
)

find_package (PCL REQUIRED COMPONENTS
	common
	io
)

set (CMAKE_BUILD_TYPE Debug)
set_property (GLOBAL 
        PROPERTY COMPILE_DEFINITIONS DEBUG=1
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vislab_3dv_node
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories (${PCL_INCLUDE_DIRS})
link_directories (${PCL_LIBRARY_DIRS})
add_definitions (${PCL_DEFINITIONS})


add_executable(vislab_3dv_node vislab_3dv_node.cpp)
target_link_libraries(vislab_3dv_node
   ${catkin_LIBRARIES}
   3dv
   dsi
   ${Boost_SYSTEM_LIBRARY}
   ${Boost_CHRONO_LIBRARY}
   ${PCL_COMMON_LIBRARY}
)


install (
	TARGETS vislab_3dv_node
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
