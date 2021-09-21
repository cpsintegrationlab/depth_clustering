# Add executables
add_executable(depth_clustering depth_clustering.cpp)

# arm64-only configurations
if(ARCH STREQUAL "arm64")
	# Set library link targets
	target_link_libraries(depth_clustering "-Wl,--no-whole-archive" dc_api)
endif()

# amd64-only configurations
if(ARCH STREQUAL "amd64")
	# Add executables
	add_executable(visualizer EXCLUDE_FROM_ALL visualizer.cpp)

	# Set library link targets
	target_link_libraries(depth_clustering dc_api)
	target_link_libraries(visualizer dc_visualization)
endif()
