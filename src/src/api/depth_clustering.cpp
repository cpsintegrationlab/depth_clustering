/*
 * depth_clustering.cpp
 *
 *  Created on: Apr 27, 2020
 *      Author: simonyu
 */

#include "api/depth_clustering.h"
#include "api/parameter_factory.h"
#include "image_labelers/diff_helpers/diff_factory.h"
#include "utils/cloud.h"
#include "utils/radians.h"
#include "utils/rich_point.h"
#include "utils/velodyne_utils.h"

using depth_clustering::Cloud;
using depth_clustering::DiffFactory;
using depth_clustering::MatFromDepthPng;
using depth_clustering::MatFromDepthTiff;
using depth_clustering::Radians;
using depth_clustering::RichPoint;

DepthClustering::Parameter::Parameter() :
		angle_clustering(10_deg), angle_ground_removal(9_deg), size_cluster_min(10), size_cluster_max(
				20000), size_smooth_window(5), bounding_box_type(BoundingBox::Type::Cube), dataset_file_type(
				".tiff"), log_file_name("depth_clustering_detection.json"), log(true)
{
}

DepthClustering::DepthClustering() :
		DepthClustering(Parameter())
{
}

DepthClustering::DepthClustering(const Parameter& parameter) :
		parameter_(parameter), log_path_("./")
{
}

bool
DepthClustering::initializeForApollo()
{
	projection_parameter_ = ProjectionParams::APOLLO();
	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*projection_parameter_,
			parameter_.angle_ground_removal, parameter_.size_smooth_window);
	clusterer_ = std::make_shared<ImageBasedClusterer<LinearImageLabeler<>>>(
			parameter_.angle_clustering, parameter_.size_cluster_min, parameter_.size_cluster_max);
	bounding_box_ = std::make_shared<BoundingBox>(parameter_.bounding_box_type);
	logger_ = std::make_shared<Logger>(parameter_.log);

	clusterer_->SetDiffType(DiffFactory::DiffType::ANGLES);
	logger_->setBoundingBox(bounding_box_);

	depth_ground_remover_->AddClient(clusterer_.get());
	clusterer_->AddClient(bounding_box_.get());

	return true;
}

bool
DepthClustering::initializeForDataset(std::string& dataset_path)
{
	if (dataset_path[dataset_path.size() - 1] != '/')
	{
		dataset_path += "/";
	}

	log_path_ = dataset_path;

	parameter_factory_ = std::make_shared<ParameterFactory>(dataset_path);
	parameter_ = parameter_factory_->getDepthClusteringParameter();
	projection_parameter_ = parameter_factory_->getLidarProjectionParameter();

	if (!projection_parameter_)
	{
		return false;
	}

	folder_reader_ = std::make_shared<FolderReader>(dataset_path, parameter_.dataset_file_type,
			FolderReader::Order::SORTED);

	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*projection_parameter_,
			parameter_.angle_ground_removal, parameter_.size_smooth_window);
	clusterer_ = std::make_shared<ImageBasedClusterer<LinearImageLabeler<>>>(
			parameter_.angle_clustering, parameter_.size_cluster_min, parameter_.size_cluster_max);
	bounding_box_ = std::make_shared<BoundingBox>(parameter_.bounding_box_type);
	camera_projection_ = std::make_shared<CameraProjection>(parameter_factory_->getCameraProjectionParameter());
	logger_ = std::make_shared<Logger>(parameter_.log);

	clusterer_->SetDiffType(DiffFactory::DiffType::ANGLES);
	logger_->setBoundingBox(bounding_box_);

	depth_ground_remover_->AddClient(clusterer_.get());
	clusterer_->AddClient(bounding_box_.get());

	return true;
}

void
DepthClustering::processForApollo(const std::string& frame_name,
		const std::vector<Eigen::Vector3f>& point_cloud)
{
	Cloud::Ptr cloud(new Cloud);

	for (const auto &point_eigen : point_cloud)
	{
		RichPoint point_rich;

		point_rich.x() = point_eigen.x();
		point_rich.y() = point_eigen.y();
		point_rich.z() = point_eigen.z();

		cloud->push_back(point_rich);
	}

	cloud->InitProjection(*projection_parameter_);

	depth_ground_remover_->OnNewObjectReceived(*cloud, 0);

	logger_->logBoundingBoxFrame(frame_name, parameter_.bounding_box_type);
	bounding_box_->clearFrame();
}

void
DepthClustering::processForDataset()
{
	for (const auto &path : folder_reader_->GetAllFilePaths())
	{
		cv::Mat depth_image;
		std::string frame_name = "";
		std::stringstream ss(path);

		while (std::getline(ss, frame_name, '/'));

		if (parameter_.dataset_file_type == ".png")
		{
			depth_image = MatFromDepthPng(path);
		}
		else if (parameter_.dataset_file_type == ".tiff")
		{
			depth_image = MatFromDepthTiff(path);
		}
		else
		{
			std::cout << "Unknown data type. Skip." << std::endl;
			return;
		}

		auto cloud = Cloud::FromImage(depth_image, *projection_parameter_);

		std::cout << std::endl << "[INFO]: Processing \"" << frame_name << "\"." << std::endl;

		depth_ground_remover_->OnNewObjectReceived(*cloud, 0);

		camera_projection_->projectBoundingBoxFrame(parameter_.bounding_box_type);
		logger_->logBoundingBoxFrame(frame_name, parameter_.bounding_box_type);

		bounding_box_->clearFrame();
	}
}

void
DepthClustering::finish()
{
	logger_->writeBoundingBoxLog(log_path_, parameter_.log_file_name);
}
