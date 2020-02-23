# Set global variables
cmake_minimum_required(VERSION 3.1)
project(depth_clustering)

# Set compiler parameters
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -fPIC -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Set CMake parameters
cmake_policy(SET CMP0045 OLD)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/../cmake")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../install/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../install/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../install/bin)

# Find packages
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem regex program_options)
find_package(PCL QUIET COMPONENTS common io)
find_package(QGLViewer REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Xml OpenGL Gui Widgets)

# If PCL found
if(${PCL_FOUND})
	add_definitions(-DPCL_FOUND)
endif()

# Set Boost parameters
set(Boost_USE_STATIC_LIBS OFF)

# Set Qt libraries
set(
	QT_LIBRARIES
	${Qt5Widgets_LIBRARIES}
	${Qt5Core_LIBRARIES}
	${Qt5Gui_LIBRARIES}
    ${Qt5Xml_LIBRARIES}
    ${Qt5OpenGL_LIBRARIES}
)

# Set include directories
include_directories(
	src
  	${PROJECT_BINARY_DIR}/src
	${Qt5Core_INCLUDE_DIRS}
    ${Qt5Xml_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5OpenGL_INCLUDE_DIRS}
  	SYSTEM ${EIGEN3_INCLUDE_DIR}
  	SYSTEM ${Boost_INCLUDE_DIRS}
  	SYSTEM ${PCL_INCLUDE_DIRS}
  	SYSTEM ${OpenCV_INCLUDE_DIRS}
  	SYSTEM ${OpenGL_INCLUDE_DIRS}
  	SYSTEM ${QGLVIEWER_INCLUDE_DIR}
)

# Add subdirectories
add_subdirectory(src)