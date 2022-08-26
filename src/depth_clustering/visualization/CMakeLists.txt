# Set package parameters
set(OpenGL_GL_PREFERENCE GLVND)

# Find packages
find_package(OpenGL QUIET)
find_package(QGLViewer QUIET)
find_package(Qt5 QUIET COMPONENTS Core Xml OpenGL Gui Widgets)

if(${OPENGL_FOUND} AND ${QGLViewer_FOUND} AND ${Qt5_FOUND})
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
		dc_visualization SHARED
		drawables/drawable_cloud.cpp
		drawables/drawable_cube.cpp
		drawables/drawable_polygon3d.cpp
		utils/utils.cpp
		viewer/viewer.cpp
		${QT_SOURCES}
		${QT_HEADERS_MOC}
		${QT_UI_FILES_MOC}
	)

	# Set library link targets
	target_link_libraries(
		dc_visualization
		dc_api
		${OPENGL_gl_LIBRARY}
		${OPENGL_glu_LIBRARY}
		${QGLVIEWER_LIBRARY}
		${QT_LIBRARIES}
	)
endif()