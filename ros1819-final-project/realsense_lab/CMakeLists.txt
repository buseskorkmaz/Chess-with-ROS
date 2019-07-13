cmake_minimum_required(VERSION 2.8.3)
project(realsense_lab)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	aruco_ros
	std_msgs
	geometry_msgs
	tf2_ros
	std_srvs
)

catkin_package(
	DEPENDS
		roscpp
		aruco_ros
)

include_directories(include
	${catkin_INCLUDE_DIRS}
)

add_executable(cameras_calibration
	src/cameras_calibration.cpp
)

target_link_libraries(cameras_calibration
	${catkin_LIBRARIES}
)
