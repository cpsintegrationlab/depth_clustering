# Add libraries
add_library(
	dc_image_labelers SHARED
	diff_helpers/angle_diff.cpp
	diff_helpers/line_dist_diff.cpp
	abstract_image_labeler.cpp
)