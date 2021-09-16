# Add libraries
add_library(
	drawable SHARED EXCLUDE_FROM_ALL
	drawable_cloud.cpp
	drawable_polygon3d.cpp
	drawable_cube.cpp
)

# Set library link targets
target_link_libraries(
	drawable
	${OpenCV_LIBS}
	${OPENGL_gl_LIBRARY}
	${OPENGL_glu_LIBRARY}
)
