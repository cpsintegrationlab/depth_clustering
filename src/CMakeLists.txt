# Set global variables
cmake_minimum_required(VERSION 2.8)
project(depth_clustering)

# Set compiler parameters
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -Wno-unused-parameter -Wno-int-in-bool-context -Wno-alloc-size-larger-than=9223372036854775807 -static-libstdc++ -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Set CMake module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/../cmake")

# Set debug installation paths
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
	set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../install/depth_clustering/debug/lib)
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../install/depth_clustering/debug/lib)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../install/depth_clustering/debug/bin)
endif (CMAKE_BUILD_TYPE STREQUAL "Debug")

# Set release installation paths
if (CMAKE_BUILD_TYPE STREQUAL "Release")
	set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../install/depth_clustering/release/lib)
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../install/depth_clustering/release/lib)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../install/depth_clustering/release/bin)
endif (CMAKE_BUILD_TYPE STREQUAL "Release")

# Set package parameters
set(Boost_NO_SYSTEM_PATHS TRUE)
set(BOOST_ROOT ${PROJECT_SOURCE_DIR}/../install/boost)
set(EIGEN3_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/../install/eigen/include/eigen3)

# Find packages
find_package(Boost REQUIRED COMPONENTS system filesystem regex program_options)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED PATHS ${PROJECT_SOURCE_DIR}/../install/opencv NO_DEFAULT_PATH)
find_package(Threads REQUIRED)

# Set include directories
include_directories(
	depth_clustering
	${PROJECT_BINARY_DIR}/depth_clustering
	${Boost_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
)

# Add subdirectories
add_subdirectory(depth_clustering)
