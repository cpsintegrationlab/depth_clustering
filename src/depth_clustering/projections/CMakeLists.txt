# Add libraries
add_library(
	dc_projections SHARED
	cloud_projection.cpp
	projection_params.cpp
	ring_projection.cpp
	spherical_projection.cpp
)

# Set library link targets
target_link_libraries(
	dc_projections
	${OpenCV_LIBS}
)