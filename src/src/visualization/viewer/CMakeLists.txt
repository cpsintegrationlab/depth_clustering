# Add libraries
add_library(viewer SHARED EXCLUDE_FROM_ALL viewer.cpp)

# Set library link targets
target_link_libraries(
	viewer
	drawable
	${QGLVIEWER_LIBRARY}
	${QT_LIBRARIES}
)