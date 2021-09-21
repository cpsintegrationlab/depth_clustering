# Add libraries
add_library(
	dc_utils SHARED
	bbox.cpp
	cloud.cpp
	folder_reader.cpp
	rich_point.cpp
	velodyne_utils.cpp
)

# Set library link targets
target_link_libraries(
	dc_utils
	${Boost_LIBRARIES}
	${OpenCV_LIBS}
)