/*
 * api.h
 *
 *  Created on: Apr 27, 2020
 *      Author: simonyu
 */

#ifndef SRC_API_DEPTH_CLUSTERING_H_
#define SRC_API_DEPTH_CLUSTERING_H_

#include <memory>
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

	const DepthClusteringParameter&
	getParameter() const;

	const CameraProjectionParameter&
	getCameraProjectionParameter() const;

	std::shared_ptr<ProjectionParams>
	getLidarProjectionParameter() const;

	const std::string&
	getDatasetPath() const;

	Cloud::ConstPtr
	getCloudRange() const;

	Cloud::ConstPtr
	getCloudIntensity() const;

	Cloud::ConstPtr
	getCloudElongation() const;

	Cloud::ConstPtr
	getCloudConfidence() const;

	const cv::Mat
	getImageCamera(const std::string& frame_path_name_camera) const;

	const cv::Mat&
	getImageRange() const;

	const cv::Mat&
	getImageIntensity() const;

	const cv::Mat&
	getImageElongation() const;

	std::shared_ptr<BoundingBox>
	getBoundingBox() const;

	std::shared_ptr<BoundingBox::Frame<BoundingBox::Flat>>
	getBoundingBoxFrameFlat() const;

	std::shared_ptr<ImageBasedClusterer<LinearImageLabeler<>>>
	getClusterer() const;

	std::shared_ptr<FolderReader>
	getFolderReaderCamera() const;

	std::shared_ptr<FolderReader>
	getFolderReaderRange() const;

	std::shared_ptr<FolderReader>
	getFolderReaderIntensity() const;

	std::shared_ptr<FolderReader>
	getFolderReaderElongation() const;

	std::shared_ptr<DepthGroundRemover>
	getDepthGroundRemover() const;

	void
	setParameter(const DepthClusteringParameter& parameter);

	bool
	initializeForApollo();

	bool
	initializeForDataset(const std::string& dataset_path,
			const std::string& global_config_path = "", const bool& second_return = false);

	void
	processOneRangeFrameForApollo(const std::string& frame_name,
			const std::vector<Eigen::Vector3f>& point_cloud);

	const std::string
	processOneRangeFrameForDataset(const std::string& frame_path_name);

	const std::string
	processOneIntensityFrameForDataset(const std::string& frame_path_name);

	const std::string
	processOneElongationFrameForDataset(const std::string& frame_path_name);

	void
	processAllGroundTruthsForDataset();

	void
	writeLogForApollo();

	void
	writeLogForDataset();

private:

	std::shared_ptr<ParameterFactory> parameter_factory_;
	std::shared_ptr<ParameterFactory> parameter_factory_global_;
	std::shared_ptr<ProjectionParams> parameter_projection_lidar_;
	std::shared_ptr<FolderReader> folder_reader_camera_;
	std::shared_ptr<FolderReader> folder_reader_range_;
	std::shared_ptr<FolderReader> folder_reader_intensity_;
	std::shared_ptr<FolderReader> folder_reader_elongation_;
	std::shared_ptr<DepthGroundRemover> depth_ground_remover_;
	std::shared_ptr<ImageBasedClusterer<LinearImageLabeler<>>> clusterer_;
	std::shared_ptr<BoundingBox> bounding_box_;
	std::shared_ptr<Logger> logger_;

	DepthClusteringParameter parameter_;
	CameraProjectionParameter parameter_projection_camera_;
	std::string dataset_path_;
	int frame_counter_;

	Cloud::Ptr cloud_range_;
	cv::Mat image_range_;
	cv::Mat image_intensity_;
	cv::Mat image_elongation_;
};

#endif /* SRC_API_DEPTH_CLUSTERING_H_ */
