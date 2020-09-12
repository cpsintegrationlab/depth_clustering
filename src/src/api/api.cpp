/*
 * api.cpp
 *
 *  Created on: Apr 27, 2020
 *      Author: simonyu
 */

#include "api/api.h"
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

DepthClustering::DepthClustering() :
		DepthClustering(DepthClusteringParameter())
{
}

DepthClustering::DepthClustering(const DepthClusteringParameter& parameter) :
		parameter_(parameter), frame_counter_(0)
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
	logger_ = std::make_shared<Logger>();

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

	parameter_factory_ = std::make_shared<ParameterFactory>(dataset_path);
	parameter_ = parameter_factory_->getDepthClusteringParameter();
	projection_parameter_ = parameter_factory_->getLidarProjectionParameter();

	if (!projection_parameter_)
	{
		return false;
	}

	auto logger_parameter = parameter_factory_->getLoggerParameter();
	logger_parameter.log_path = dataset_path;

	folder_reader_ = std::make_shared<FolderReader>(dataset_path, parameter_.dataset_file_type,
			FolderReader::Order::SORTED);

	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*projection_parameter_,
			parameter_.angle_ground_removal, parameter_.size_smooth_window);
	clusterer_ = std::make_shared<ImageBasedClusterer<LinearImageLabeler<>>>(
			parameter_.angle_clustering, parameter_.size_cluster_min, parameter_.size_cluster_max);
	bounding_box_ = std::make_shared<BoundingBox>(parameter_.bounding_box_type,
			parameter_factory_->getCameraProjectionParameter());
	logger_ = std::make_shared<Logger>(logger_parameter);

	clusterer_->SetDiffType(DiffFactory::DiffType::ANGLES);
	logger_->setBoundingBox(bounding_box_);

	depth_ground_remover_->AddClient(clusterer_.get());
	clusterer_->AddClient(bounding_box_.get());

	return true;
}

std::shared_ptr<BoundingBox::Frame<BoundingBox::Flat>>
DepthClustering::getBoundingBoxFrameFlat() const
{
	if (!bounding_box_)
	{
		return nullptr;
	}

	return bounding_box_->getFrameFlat();
}

void
DepthClustering::processOneFrameForApollo(const std::string& frame_name,
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

	bounding_box_->clearFrames();
	depth_ground_remover_->OnNewObjectReceived(*cloud, 0);

	logger_->logBoundingBoxFrame(frame_name, parameter_.bounding_box_type);
}

const std::string
DepthClustering::processOneFrameForDataset(const std::string& frame_path_name)
{
	cv::Mat depth_image;
	std::string frame_name = "";
	std::stringstream ss(frame_path_name);

	while (std::getline(ss, frame_name, '/'))
	{
	}

	if (parameter_.dataset_file_type == ".png")
	{
		depth_image = MatFromDepthPng(frame_path_name);
	}
	else if (parameter_.dataset_file_type == ".tiff")
	{
		depth_image = MatFromDepthTiff(frame_path_name);
	}
	else
	{
		std::cout << "[INFO]: Unknown data type. Skip." << std::endl;
		return "";
	}

	auto cloud = Cloud::FromImage(depth_image, *projection_parameter_);

	std::cout << std::endl << "[INFO]: Processing \"" << frame_name << "\"." << std::endl;

	bounding_box_->clearFrames();
	depth_ground_remover_->OnNewObjectReceived(*cloud, 0);
	bounding_box_->produceFrameFlat();

	return frame_name;
}

const std::string
DepthClustering::processNextFrameForDataset()
{
	std::string frame_name = "";
	const auto &frame_paths_names = folder_reader_->GetAllFilePaths();

	if (frame_counter_ >= static_cast<int>(frame_paths_names.size()))
	{
		return frame_name;
	}

	while (frame_name == "" && frame_counter_ < static_cast<int>(frame_paths_names.size()))
	{
		frame_name = processOneFrameForDataset(frame_paths_names[frame_counter_++]);
	}

	return frame_name;
}

const std::string
DepthClustering::processLastFrameForDataset()
{
	std::string frame_name = "";
	const auto &frame_paths_names = folder_reader_->GetAllFilePaths();

	if (frame_counter_ < 0)
	{
		return frame_name;
	}

	while (frame_name == "" && frame_counter_ >= 0)
	{
		frame_name = processOneFrameForDataset(frame_paths_names[frame_counter_--]);
	}

	return frame_name;
}

void
DepthClustering::processAllFramesForDataset()
{
	for (const auto &frame_path_name : folder_reader_->GetAllFilePaths())
	{
		const auto &frame_name = processOneFrameForDataset(frame_path_name);

		if (frame_name != "")
		{
			logger_->logBoundingBoxFrame(frame_name, parameter_.bounding_box_type);
			logger_->logBoundingBoxFrameFlat(frame_name);
		}
	}
}

void
DepthClustering::logForApollo()
{
	logger_->writeBoundingBoxLog(parameter_.bounding_box_type);
}

void
DepthClustering::logForDataset()
{
	logger_->writeBoundingBoxLog(parameter_.bounding_box_type);
	logger_->writeBoundingBoxLog(BoundingBox::Type::Flat);
}
