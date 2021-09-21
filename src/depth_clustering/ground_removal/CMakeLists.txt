# Add libraries
add_library(
	dc_ground_removal SHARED
	depth_ground_remover.cpp
)

# Set library link targets
target_link_libraries(
	dc_ground_removal
	dc_communication
	dc_image_labelers
	dc_utils
	${OpenCV_LIBS}
)