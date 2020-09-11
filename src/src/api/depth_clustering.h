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

#include "api/depth_clustering_parameter.h"
#include "clusterers/image_based_clusterer.h"
#include "ground_removal/depth_ground_remover.h"
#include "image_labelers/linear_image_labeler.h"
#include "post_processing/logger.h"
#include "projections/projection_params.h"
#include "utils/folder_reader.h"

using depth_clustering::DepthGroundRemover;
using depth_clustering::FolderReader;
using depth_clustering::ImageBasedClusterer;
using depth_clustering::LinearImageLabeler;
using depth_clustering::Logger;
using depth_clustering::ProjectionParams;

class ParameterFactory;

class DepthClustering
{
public:

	DepthClustering();

	DepthClustering(const DepthClusteringParameter& parameter);

	bool
	initializeForApollo();

	bool
	initializeForDataset(std::string& dataset_path);

	void
	processForApollo(const std::string& frame_name,
			const std::vector<Eigen::Vector3f>& point_cloud);

	void
	processForDataset();

	void
	finishForApollo();

	void
	finishForDataset();

private:

	DepthClusteringParameter parameter_;

	std::shared_ptr<ParameterFactory> parameter_factory_;
	std::unique_ptr<ProjectionParams> projection_parameter_;
	std::shared_ptr<FolderReader> folder_reader_;
	std::shared_ptr<DepthGroundRemover> depth_ground_remover_;
	std::shared_ptr<ImageBasedClusterer<LinearImageLabeler<>>> clusterer_;
	std::shared_ptr<BoundingBox> bounding_box_;
	std::shared_ptr<Logger> logger_;
};

#endif /* SRC_API_DEPTH_CLUSTERING_H_ */
