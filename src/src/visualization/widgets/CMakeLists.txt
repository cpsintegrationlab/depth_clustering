# Set CMake parameters
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Set Qt parameters
set(
	QT_SOURCES
    base_viewer_widget.cpp
    opengl_folder_player.cpp
)
set(
	QT_HEADERS
    base_viewer_widget.h
    opengl_folder_player.h
)
set(
	QT_UI_FILES
    ui/opengl_folder_player.ui
)

# Configure Qt 5 wraps
QT5_WRAP_CPP(QT_HEADERS_MOC ${QT_HEADERS})
QT5_WRAP_UI(QT_UI_FILES_MOC ${QT_UI_FILES})

# Add libraries
add_library(
	qt_widgets SHARED EXCLUDE_FROM_ALL
	${QT_SOURCES}
	${QT_UI_FILES_MOC}
	${QT_HEADERS_MOC}
)

# Set library link targets
target_link_libraries(
	qt_widgets
	viewer
	drawable
	qt_utils
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

