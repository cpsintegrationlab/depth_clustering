/*
 * depth_clustering.h
 *
 *  Created on: Apr 27, 2020
 *      Author: simonyu
 */

#ifndef SRC_API_DEPTH_CLUSTERING_H_
#define SRC_API_DEPTH_CLUSTERING_H_

#include <memory>
#include <thread>
#include <string>

#include "clusterers/image_based_clusterer.h"
#include "ground_removal/depth_ground_remover.h"
#include "image_labelers/linear_image_labeler.h"
#include "post_processing/bounding_box.h"
#include "post_processing/logger.h"
#include "projections/projection_params.h"
#include "utils/folder_reader.h"
#include "utils/radians.h"

using depth_clustering::BoundingBox;
using depth_clustering::DepthGroundRemover;
using depth_clustering::FolderReader;
using depth_clustering::ImageBasedClusterer;
using depth_clustering::LinearImageLabeler;
using depth_clustering::Logger;
using depth_clustering::ProjectionParams;
using depth_clustering::Radians;

class DepthClustering
{
public:

	struct Parameter
	{
		Radians angle_clustering;
		Radians angle_ground_removal;
		int size_cluster_min;
		int size_cluster_max;
		int size_smooth_window;
		BoundingBox::Type bounding_box_type;
		bool log;

		Parameter();
	};

	DepthClustering();

	DepthClustering(const Parameter& parameter);

	bool
	initializeForApollo();

	bool
	initializeForDataset(const std::string& dataset_path, const std::string& dataset_file_type);

	void
	processForApollo(const std::string& frame_name,
			const std::vector<Eigen::Vector3f>& point_cloud);

	void
	processForDataset();

	void
	finish();

private:

	Parameter parameter_;

	std::string dataset_file_type_;
	std::string log_path_;
	std::string log_file_name_;

	std::shared_ptr<FolderReader> folder_reader_data_;
	std::shared_ptr<FolderReader> folder_reader_config_;
	std::unique_ptr<ProjectionParams> projection_parameter_;
	std::shared_ptr<DepthGroundRemover> depth_ground_remover_;
	std::shared_ptr<ImageBasedClusterer<LinearImageLabeler<>>> clusterer_;
	std::shared_ptr<BoundingBox> bounding_box_;
	std::shared_ptr<BoundingBox::Frame<BoundingBox::Cube>> bounding_box_frame_cube_;
	std::shared_ptr<BoundingBox::Frame<BoundingBox::Polygon>> bounding_box_frame_polygon_;
	std::shared_ptr<Logger> logger_;
};

#endif /* SRC_API_DEPTH_CLUSTERING_H_ */
