# Add executables
add_executable(camera_visualizer camera_visualizer.cpp)
add_executable(depth_clustering depth_clustering.cpp)

# Set library link targets
target_link_libraries(camera_visualizer api)
target_link_libraries(depth_clustering api)