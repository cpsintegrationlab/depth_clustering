# Add libraries
add_library(
	api SHARED
	api.cpp
	parameter_factory.cpp
)

# Set library link targets
target_link_libraries(
	api
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