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
#include "projections/projection_params.h"
#include "utils/folder_reader.h"
#include "utils/radians.h"

using depth_clustering::DepthGroundRemover;
using depth_clustering::FolderReader;
using depth_clustering::ImageBasedClusterer;
using depth_clustering::LinearImageLabeler;
using depth_clustering::BoundingBox;
using depth_clustering::ProjectionParams;
using depth_clustering::Radians;

class DepthClustering
{
public:

	DepthClustering();

	DepthClustering(std::string data_type, BoundingBox::Type bounding_box_type,
			int size_cluster_min, int size_cluster_max, int size_smooth_window,
			float angle_clustering, float angle_ground_removal, bool log_apollo, bool log_data);

	bool
	initApollo(const BoundingBox::Type& bounding_box_type);

	bool
	initDataset(const std::string& data_folder, const std::string& data_type,
			const BoundingBox::Type& bounding_box_type);

	void
	processApollo(const std::string& frame_name, const std::vector<Eigen::Vector3f>& point_cloud);

	void
	processDataset();

	void
	finish();

private:

	void
	resetBoundingBox(bool& log);

	void
	clearBoundingBoxFrame();

	void
	storeBoundingBoxFrame();

	std::string data_type_;
	BoundingBox::Type bounding_box_type_;
	Radians angle_clustering_;
	Radians angle_ground_removal_;
	int size_cluster_min_;
	int size_cluster_max_;
	int size_smooth_window_;
	bool log_apollo_;
	bool log_data_;

	std::shared_ptr<FolderReader> folder_reader_data_;
	std::shared_ptr<FolderReader> folder_reader_config_;
	std::unique_ptr<ProjectionParams> projection_parameter_;
	std::shared_ptr<DepthGroundRemover> depth_ground_remover_;
	std::shared_ptr<ImageBasedClusterer<LinearImageLabeler<>>> clusterer_;
	std::unique_ptr<BoundingBox> bounding_box_;

	BoundingBox::Frame<BoundingBox::Cube> bounding_box_frame_cube_;
	BoundingBox::Frame<BoundingBox::Polygon> bounding_box_frame_polygon_;
	std::vector<BoundingBox::Frame<BoundingBox::Cube>> bounding_box_frames_cube_;
	std::vector<BoundingBox::Frame<BoundingBox::Polygon>> bounding_box_frames_polygon_;
};

#endif /* SRC_API_DEPTH_CLUSTERING_H_ */
