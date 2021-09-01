/*
 * api.h
 *
 *  Created on: Apr 27, 2020
 *      Author: simonyu
 */

#ifndef SRC_API_DEPTH_CLUSTERING_H_
#define SRC_API_DEPTH_CLUSTERING_H_

#include <memory>
#include <thread>
#include <string>

#include "api/parameter.h"
#include "clusterers/image_based_clusterer.h"
#include "ground_removal/depth_ground_remover.h"
#include "image_labelers/linear_image_labeler.h"
#include "post_processing/logger.h"
#include "projections/projection_params.h"
#include "utils/folder_reader.h"

using depth_clustering::Cloud;
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
	initializeForDataset(std::string dataset_path);

	const DepthClusteringParameter&
	getParameter() const;

	const std::string&
	getDatasetPath() const;

	const cv::Mat&
	getCurrentDepthImage() const;

	Cloud::ConstPtr
	getCurrentCloud() const;

	std::shared_ptr<BoundingBox>
	getBoundingBox() const;

	std::shared_ptr<BoundingBox::Frame<BoundingBox::Flat>>
	getBoundingBoxFrameFlat() const;

	std::shared_ptr<ImageBasedClusterer<LinearImageLabeler<>>>
	getClusterer() const;

	std::shared_ptr<FolderReader>
	getFolderReader() const;

	std::shared_ptr<ProjectionParams>
	getProjectionParameter() const;

	std::shared_ptr<DepthGroundRemover>
	getDepthGroundRemover() const;

	void
	setParameter(const DepthClusteringParameter& parameter);

	void
	processOneFrameForApollo(const std::string& frame_name,
			const std::vector<Eigen::Vector3f>& point_cloud);

	const std::string
	processOneFrameForDataset(const std::string& frame_path_name);

	const std::string
	processNextFrameForDataset();

	const std::string
	processLastFrameForDataset();

	void
	processAllFramesForDataset();

	void
	processGroundTruthForDataset();

	void
	logForApollo();

	void
	logForDataset();

private:

	std::shared_ptr<ParameterFactory> parameter_factory_;
	std::shared_ptr<ProjectionParams> projection_parameter_;
	std::shared_ptr<FolderReader> folder_reader_;
	std::shared_ptr<DepthGroundRemover> depth_ground_remover_;
	std::shared_ptr<ImageBasedClusterer<LinearImageLabeler<>>> clusterer_;
	std::shared_ptr<BoundingBox> bounding_box_;
	std::shared_ptr<Logger> logger_;

	DepthClusteringParameter parameter_;
	cv::Mat current_depth_image_;
	Cloud::Ptr current_cloud_;
	std::string dataset_path_;
	int frame_counter_;
};

#endif /* SRC_API_DEPTH_CLUSTERING_H_ */
