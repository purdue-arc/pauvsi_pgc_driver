cmake_minimum_required(VERSION 2.8.3)
project(pauvsi_pgc_driver)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

find_package(OpenCV REQUIRED)

find_library(POINTGREY_LIB flycapture)
if(NOT POINTGREY_LIB)
  message(STATUS "libflycapture not found in system library path")
  include(cmake/DownloadFlyCap.cmake)
  download_flycap(POINTGREY_LIB POINTGREY_INCLUDE_DIR) 
  message(STATUS "libflycapture library: ${POINTGREY_LIB}")
  message(STATUS "libflycapture include: ${POINTGREY_INCLUDE_DIR}")
  include_directories(${POINTGREY_INCLUDE_DIR})  
endif()


catkin_package(
	INCLUDE_DIRS include 
	DEPENDS eigen opencv
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(pgcDriverLib include/pgc_driver/Driver.h)
set_target_properties(pgcDriverLib PROPERTIES LINKER_LANGUAGE CXX)

add_executable(pgcDriver src/pgc_driver.cpp)
target_link_libraries(pgcDriver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} pgcDriverLib)

