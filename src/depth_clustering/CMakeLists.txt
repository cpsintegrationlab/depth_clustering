# Add subdirectories
add_subdirectory(api)
add_subdirectory(applications)
add_subdirectory(communication)
add_subdirectory(ground_removal)
add_subdirectory(image_labelers)
add_subdirectory(post_processing)
add_subdirectory(projections)
add_subdirectory(utils)

# amd64-only targets
if (ARCH STREQUAL "amd64")
  add_subdirectory(visualization)
endif()
