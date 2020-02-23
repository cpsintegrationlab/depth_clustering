# Add libraries
add_library(qt_utils utils.cpp)

# Set library link targets
target_link_libraries(
	qt_utils
	${QT_LIBRARIES}
	${OpenCV_LIBS}
)
