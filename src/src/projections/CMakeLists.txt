# Add libraries
add_library(
	projections SHARED
	projection_params.cpp
	cloud_projection.cpp
	ring_projection.cpp
	spherical_projection.cpp
)

# Set library link targets
target_link_libraries(projections ${OpenCV_LIBS})