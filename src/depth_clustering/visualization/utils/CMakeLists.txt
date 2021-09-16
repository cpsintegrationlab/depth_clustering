# Add libraries
add_library(visualization_utils SHARED EXCLUDE_FROM_ALL utils.cpp)

# Set library link targets
target_link_libraries(
	visualization_utils
	${QT_LIBRARIES}
	${OpenCV_LIBS}
)
