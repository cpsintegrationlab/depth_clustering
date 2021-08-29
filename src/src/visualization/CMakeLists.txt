# Find packages
find_package(OpenGL REQUIRED)
find_package(QGLViewer REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Xml OpenGL Gui Widgets)

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
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Xml_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5OpenGL_INCLUDE_DIRS}
    SYSTEM ${OpenGL_INCLUDE_DIRS}
    SYSTEM ${QGLVIEWER_INCLUDE_DIR}
)

# Set CMake parameters
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Set Qt parameters
set(
	QT_SOURCES
    visualization.cpp
)
set(
	QT_HEADERS
    visualization.h
)
set(
	QT_UI_FILES
    ui/visualization.ui
)

# Configure Qt 5 wraps
QT5_WRAP_CPP(QT_HEADERS_MOC ${QT_HEADERS})
QT5_WRAP_UI(QT_UI_FILES_MOC ${QT_UI_FILES})

# Add libraries
add_library(
	visualization SHARED EXCLUDE_FROM_ALL
	${QT_SOURCES}
	${QT_UI_FILES_MOC}
	${QT_HEADERS_MOC}
)

# Set library link targets
target_link_libraries(
	visualization
	viewer
	drawable
	visualization_utils
	identifiable
	ground_remove
	image_labeler
	velodyne_utils
	folder_reader
	bbox
	post_processing
	${OpenCV_LIBS}
	${QGLVIEWER_LIBRARY}
	${QT_LIBRARIES}
)

# Add subdirectories             
add_subdirectory(drawables)
add_subdirectory(viewer)
add_subdirectory(utils)