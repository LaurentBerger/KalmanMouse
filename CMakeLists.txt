cmake_minimum_required(VERSION 2.8)

PROJECT (KalmanMouse)

find_package(OpenCV  REQUIRED)


file(GLOB KalmanMouse_SRCS
    "*.h"
    "*.cpp")
ADD_EXECUTABLE (KalmanMouse ${KalmanMouse_SRCS})



if (OpenCV_FOUND)
	include_directories( ${OpenCV_INCLUDE_DIRS} )
#	include_directories("${OPENCV_CONFIG_PATH}/include")
	target_link_libraries( KalmanMouse ${OpenCV_LIBS} )
else (OpenCV_FOUND)
message("PB->OPENCV = ${OpenCV_INCLUDE_DIRS}")
endif (OpenCV_FOUND)

