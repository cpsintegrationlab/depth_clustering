# Set global variables
cmake_minimum_required(VERSION 2.8)
project(depth_clustering)

# Set compiler parameters
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -fPIC -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Set CMake parameters
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/../cmake")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../install/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../install/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../install/bin)

# Find packages
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem regex program_options)
find_package(PCL QUIET COMPONENTS common io)

# If PCL found
if(${PCL_FOUND})
    add_definitions(-DPCL_FOUND)
endif()

# Set Boost parameters
set(Boost_USE_STATIC_LIBS OFF)

# Set include directories
include_directories(
	src
  	${PROJECT_BINARY_DIR}/src
  	SYSTEM ${EIGEN3_INCLUDE_DIR}
  	SYSTEM ${Boost_INCLUDE_DIRS}
	SYSTEM ${PCL_INCLUDE_DIRS}
  	SYSTEM ${OpenCV_INCLUDE_DIRS}
)

# Add subdirectories
add_subdirectory(src)
