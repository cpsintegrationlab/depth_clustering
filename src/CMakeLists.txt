# Set global parameters
cmake_minimum_required(VERSION 2.8)
project(depth_clustering)

# Set CMake parameters
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/../cmake")

# Set compiler parameters
set(CMAKE_C_FLAGS "-Wall -Wextra -static-libc -fPIC")
set(CMAKE_C_FLAGS_DEBUG "-g -O0")
set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -Wno-unused-parameter -Wno-int-in-bool-context -Wno-alloc-size-larger-than=9223372036854775807 -static-libstdc++ -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# arm64-only configurations
if(ARCH STREQUAL "arm64")
	set(CMAKE_EXE_LINKER_FLAGS "-Wl,--whole-archive")
endif()

# Set installation paths
string(TOLOWER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_PATH)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../install/${ARCH}/depth_clustering/${CMAKE_BUILD_TYPE_PATH}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../install/${ARCH}/depth_clustering/${CMAKE_BUILD_TYPE_PATH}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../install/${ARCH}/depth_clustering/${CMAKE_BUILD_TYPE_PATH}/lib)

# Set package parameters
set(Boost_NO_SYSTEM_PATHS TRUE)
set(BOOST_ROOT ${PROJECT_SOURCE_DIR}/../install/${ARCH}/boost)
set(EIGEN_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/../install/${ARCH}/eigen/include/eigen3)

# Find packages
find_package(Boost REQUIRED COMPONENTS system filesystem regex program_options)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED PATHS ${PROJECT_SOURCE_DIR}/../install/${ARCH}/opencv NO_DEFAULT_PATH)

# Set include directories
include_directories(
	depth_clustering
	${PROJECT_BINARY_DIR}/depth_clustering
	${Boost_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

# Set installation targets
install(DIRECTORY ${PROJECT_SOURCE_DIR}/../config/ DESTINATION config/depth_clustering)

# Add subdirectories
add_subdirectory(depth_clustering)
