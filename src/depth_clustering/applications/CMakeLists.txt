# Add executables
add_executable(camera_visualizer camera_visualizer.cpp)
add_executable(depth_clustering depth_clustering.cpp)
add_executable(ground_truth_projection ground_truth_projection.cpp)

# arm64-only configurations
if(ARCH STREQUAL "arm64")
	# Set library link targets
	target_link_libraries(depth_clustering "-Wl,--no-whole-archive" api)
	target_link_libraries(camera_visualizer "-Wl,--no-whole-archive" api)
	target_link_libraries(ground_truth_projection "-Wl,--no-whole-archive" api)
endif()

# amd64-only configurations
if(ARCH STREQUAL "amd64")
	# Add executables
	add_executable(lidar_visualizer EXCLUDE_FROM_ALL lidar_visualizer.cpp)

	# Set library link targets
	target_link_libraries(depth_clustering api)
	target_link_libraries(camera_visualizer api)
	target_link_libraries(ground_truth_projection api)
	target_link_libraries(lidar_visualizer visualization)
endif()
