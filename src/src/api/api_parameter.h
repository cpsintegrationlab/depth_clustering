/*
 * depth_clustering_parameter.h
 *
 *  Created on: Sep 11, 2020
 *      Author: simonyu
 */

#ifndef SRC_API_DEPTH_CLUSTERING_PARAMETER_H_
#define SRC_API_DEPTH_CLUSTERING_PARAMETER_H_

#include "post_processing/bounding_box.h"
#include "utils/radians.h"

using depth_clustering::BoundingBox;
using depth_clustering::Radians;

struct DepthClusteringParameter
{
	Radians angle_clustering;
	Radians angle_ground_removal;
	int size_cluster_min;
	int size_cluster_max;
	int size_smooth_window;
	BoundingBox::Type bounding_box_type;
	std::string dataset_file_type;

	DepthClusteringParameter() :
			angle_clustering(10_deg), angle_ground_removal(9_deg), size_cluster_min(10), size_cluster_max(
					20000), size_smooth_window(5), bounding_box_type(BoundingBox::Type::Cube), dataset_file_type(
					".tiff")
	{
	}
};

#endif /* SRC_API_DEPTH_CLUSTERING_PARAMETER_H_ */
