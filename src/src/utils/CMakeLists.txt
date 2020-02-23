# Add libraries
add_library(cloud SHARED cloud.cpp rich_point.cpp)
add_library(velodyne_utils SHARED velodyne_utils.cpp)
add_library(folder_reader SHARED folder_reader.cpp)
add_library(bbox bbox.cpp)

# Set library link targets
target_link_libraries(
	cloud
	projections
	${OpenCV_LIBS}
	${PCL_LIBRARIES}
)
target_link_libraries(
	velodyne_utils
	projections
	cloud
	${PCL_LIBRARIES}
	${OpenCV_LIBS}
)
target_link_libraries(folder_reader ${Boost_LIBRARIES})
target_link_libraries(bbox cloud)