# Add executables
add_executable(camera_visualizer camera_visualizer.cpp)
add_executable(depth_clustering depth_clustering.cpp)
add_executable(ground_truth_projection ground_truth_projection.cpp)

# Set library link targets
target_link_libraries(depth_clustering api)
target_link_libraries(camera_visualizer api)
target_link_libraries(ground_truth_projection api)

# amd64-only targets
if (ARCH STREQUAL "amd64")
  add_executable(lidar_visualizer EXCLUDE_FROM_ALL lidar_visualizer.cpp)
  target_link_libraries(lidar_visualizer visualization)
endif()
