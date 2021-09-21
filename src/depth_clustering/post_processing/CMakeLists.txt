# Add libraries
add_library(
	dc_post_processing SHARED
	bounding_box.cpp
	camera_projection.cpp
	logger.cpp
)

# Set library link targets
target_link_libraries(
	dc_post_processing
	${Boost_LIBRARIES}
)