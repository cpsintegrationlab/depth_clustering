# Add libraries
add_library(viewer SHARED viewer.cpp)

# Set library link targets
target_link_libraries(
	viewer
	drawable
	${QGLVIEWER_LIBRARY}
	${QT_LIBRARIES}
	${OPENGL_gl_LIBRARY}
	${OPENGL_glu_LIBRARY}
)