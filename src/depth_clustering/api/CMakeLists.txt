# Add libraries
add_library(
	dc_api SHARED
	api.cpp
	parameter_factory.cpp
)

# Set library link targets
target_link_libraries(
	dc_api
	dc_communication
	dc_ground_removal
	dc_image_labelers
	dc_post_processing
	dc_projections
	dc_utils
	${Boost_LIBRARIES}
	${OpenCV_LIBS}
)