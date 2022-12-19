cmake_minimum_required(VERSION 2.8.12)

project(pointgrey_ladybug)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation
	tf
	cv_bridge
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -Wall -g ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)

catkin_package()

###########
## Build ##
###########


if(EXISTS "/usr/include/ladybug")
	include_directories(
		/usr/include/ladybug
		src/ladybug/
		${catkin_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
	)
	add_executable(ladybug_camera
		src/ladybug/ladybug_driver.cpp
	)
	target_link_libraries(ladybug_camera
		${catkin_LIBRARIES}
		${OpenCV_LIBS}
		/usr/lib/ladybug/libflycapture.so
		/usr/lib/ladybug/libladybug.so
		# /usr/lib/x86_64-linux-gnu/libavcodec.so
		# /usr/lib/x86_64-linux-gnu/libavformat.so
		# /usr/lib/x86_64-linux-gnu/libavutil.so
	)
else()
	message(ERROR "'SDK for Ladybug' is not installed. 'ladybug_camera' will not be built.")
endif()