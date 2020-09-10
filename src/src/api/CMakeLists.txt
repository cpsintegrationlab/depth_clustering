# Add libraries
add_library(depth_clustering SHARED depth_clustering.cpp)

# Set library link targets
target_link_libraries(
	depth_clustering
	identifiable
	image_labeler
	velodyne_utils
	projections
	ground_remove
	folder_reader
	post_processing
	${Boost_LIBRARIES}
	${OpenCV_LIBS}
	${CMAKE_THREAD_LIBS_INIT}
)

# Add executables
add_executable(depth_clustering_bin main.cpp)

# Set library link targets
target_link_libraries(depth_clustering_bin depth_clustering)

# Set target properties 
set_target_properties(depth_clustering_bin PROPERTIES OUTPUT_NAME depth_clustering)