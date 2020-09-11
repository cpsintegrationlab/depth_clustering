# Add executables
add_executable(depth_clustering depth_clustering.cpp)

# Set library link targets
target_link_libraries(depth_clustering api)