cmake_minimum_required(VERSION 3.0)
project(AllegroHand)

Set (AllegroHand_VERSION_MAJOR 1)
Set (AllegroHand_VERSION_MINOR 0)

set (CMAKE_CXX_FLAGS "-O2 -Wall -std=c++11")

FIND_PACKAGE(PkgConfig REQUIRED)

# eigen3
set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR})
if(NOT EIGEN3_INCLUDE_DIR)
	find_package(Eigen3 QUIET)
	# TODO: Find automatic way to find for Ubuntu 14.04
	if(NOT EIGEN3_INCLUDE_DIR)
		set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
	endif()
endif()
include_directories(${EIGEN3_INCLUDE_DIR})

# hiredis
find_library(HIREDIS_LIBRARY hiredis)

# jsoncpp
pkg_check_modules(JSONCPP jsoncpp)
include_directories(${JSONCPP_INCLUDE_DIRS})
link_directories(${JSONCPP_LIBRARY_DIRS})
set(JSONCPP_LIBRARY ${JSONCPP_LIBRARIES})
include_directories(${JSONCPP_INCLUDE_DIRS})

# create libraries list 
set(REDIS_LIBRARY
	${JSONCPP_LIBRARY}
	${HIREDIS_LIBRARY})

add_subdirectory (grasp)
