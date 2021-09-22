/*
 * depth_clustering_parameter.h
 *
 *  Created on: Sep 11, 2020
 *      Author: simonyu
 */

#ifndef SRC_API_PARAMETER_H_
#define SRC_API_PARAMETER_H_

#include "image_labelers/diff_helpers/diff_factory.h"
#include "post_processing/bounding_box.h"
#include "utils/radians.h"

using depth_clustering::BoundingBox;
using depth_clustering::DiffFactory;
using depth_clustering::Radians;

struct DepthClusteringParameter
{
	double distance_clustering;
	Radians angle_clustering;
	Radians angle_ground_removal;
	int size_cluster_min;
	int size_cluster_max;
	int size_smooth_window;
	bool use_camera_fov;
	BoundingBox::Type bounding_box_type;
	DiffFactory::DiffType difference_type;
	std::string dataset_file_type;
	std::string dataset_name;
	std::string ground_truth_cube_file_name;
	std::string ground_truth_flat_file_name;

	DepthClusteringParameter() :
			distance_clustering(0.17), angle_clustering(10_deg), angle_ground_removal(9_deg), size_cluster_min(
					10), size_cluster_max(20000), size_smooth_window(5), use_camera_fov(true), bounding_box_type(
					BoundingBox::Type::Cube), difference_type(
					DiffFactory::DiffType::ANGLES_PRECOMPUTED), dataset_file_type(".tiff"), dataset_name(
					""), ground_truth_cube_file_name("waymo_ground_truth_cube.json"), ground_truth_flat_file_name(
					"depth_clustering_ground_truth_flat.json")
	{
	}
};

#endif /* SRC_API_PARAMETER_H_ */
