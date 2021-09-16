# Add libraries
add_library(image_labeler SHARED abstract_image_labeler.cpp)

# Set library link targets
target_link_libraries(image_labeler difference)

# Add subdirectories
add_subdirectory(diff_helpers)