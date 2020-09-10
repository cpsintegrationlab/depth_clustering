/*
 * depth_clustering.cpp
 *
 *  Created on: Apr 27, 2020
 *      Author: simonyu
 */

#include "api/depth_clustering.h"

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

DepthClustering::DepthClustering()
{
	dataset_file_type_ = ".tiff";
	bounding_box_type_ = BoundingBox::Type::Cube;

	size_cluster_min_ = 10;
	size_cluster_max_ = 20000;
	size_smooth_window_ = 5;
	angle_clustering_ = 10_deg;
	angle_ground_removal_ = 9_deg;

	log_ = true;
	log_path_ = "./";
	log_file_name_ = "detection.json";

	bounding_box_frame_cube_ = std::make_shared<BoundingBox::Frame<BoundingBox::Cube>>();
	bounding_box_frame_polygon_ = std::make_shared<BoundingBox::Frame<BoundingBox::Polygon>>();
}

DepthClustering::DepthClustering(BoundingBox::Type bounding_box_type, int size_cluster_min,
		int size_cluster_max, int size_smooth_window, float angle_clustering,
		float angle_ground_removal, bool log) :
		bounding_box_type_(bounding_box_type), size_cluster_min_(size_cluster_min), size_cluster_max_(
				size_cluster_max), size_smooth_window_(size_smooth_window), log_(log)
{
	dataset_file_type_ = ".tiff";

	angle_clustering_ = Radians
	{ Radians::IsRadians
	{ }, static_cast<float>(angle_clustering * M_PI / 180.0) };
	angle_ground_removal_ = Radians
	{ Radians::IsRadians
	{ }, static_cast<float>(angle_ground_removal * M_PI / 180.0) };

	log_path_ = "./";
	log_file_name_ = "detection.json";

	bounding_box_frame_cube_ = std::make_shared<BoundingBox::Frame<BoundingBox::Cube>>();
	bounding_box_frame_polygon_ = std::make_shared<BoundingBox::Frame<BoundingBox::Polygon>>();
}

bool
DepthClustering::initializeForApollo()
{
	projection_parameter_ = ProjectionParams::APOLLO();
	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*projection_parameter_,
			angle_ground_removal_, size_smooth_window_);
	bounding_box_ = std::make_shared<BoundingBox>(bounding_box_type_);
	clusterer_ = std::make_shared<ImageBasedClusterer<LinearImageLabeler<>>>(angle_clustering_,
			size_cluster_min_, size_cluster_max_);
	logger_ = std::make_shared<Logger>(log_);

	bounding_box_->setFrame(bounding_box_frame_cube_, bounding_box_frame_polygon_);
	clusterer_->SetDiffType(DiffFactory::DiffType::ANGLES);
	logger_->setBoundingBoxFrame(bounding_box_frame_cube_, bounding_box_frame_polygon_);

	depth_ground_remover_->AddClient(clusterer_.get());
	clusterer_->AddClient(bounding_box_.get());

	return true;
}

bool
DepthClustering::initializeForDataset(const std::string& dataset_path,
		const std::string& dataset_file_type)
{
	dataset_file_type_ = dataset_file_type;
	log_path_ = dataset_path;

	folder_reader_data_ = std::make_shared<FolderReader>(dataset_path, dataset_file_type_,
			FolderReader::Order::SORTED);
	folder_reader_config_ = std::make_shared<FolderReader>(dataset_path, "img.cfg");

	bounding_box_ = std::make_shared<BoundingBox>(bounding_box_type_);
	clusterer_ = std::make_shared<ImageBasedClusterer<LinearImageLabeler<>>>(angle_clustering_,
			size_cluster_min_, size_cluster_max_);
	logger_ = std::make_shared<Logger>(log_);

	bounding_box_->setFrame(bounding_box_frame_cube_, bounding_box_frame_polygon_);
	clusterer_->SetDiffType(DiffFactory::DiffType::ANGLES);
	logger_->setBoundingBoxFrame(bounding_box_frame_cube_, bounding_box_frame_polygon_);

	auto config_file_name = folder_reader_config_->GetNextFilePath();

	if (!config_file_name.empty())
	{
		projection_parameter_ = ProjectionParams::FromConfigFile(config_file_name);
	}
	else
	{
		return false;
	}

	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*projection_parameter_,
			angle_ground_removal_, size_smooth_window_);

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

	depth_ground_remover_->OnNewObjectReceived(std::make_pair(frame_name, *cloud), 0);

	logger_->logBoundingBoxFrame(frame_name, bounding_box_type_);
}

void
DepthClustering::processForDataset()
{
	for (const auto &path : folder_reader_data_->GetAllFilePaths())
	{
		cv::Mat depth_image;
		std::string frame_name = "";
		std::stringstream ss(path);

		if (dataset_file_type_ == ".png")
		{
			depth_image = MatFromDepthPng(path);
		}
		else if (dataset_file_type_ == ".tiff")
		{
			depth_image = MatFromDepthTiff(path);
		}
		else
		{
			std::cout << "Unknown data type. Skip." << std::endl;
			return;
		}

		auto cloud = Cloud::FromImage(depth_image, *projection_parameter_);

		std::cout << "[INFO]: Started processing frame." << std::endl;

		depth_ground_remover_->OnNewObjectReceived(std::make_pair(path, *cloud), 0);

		std::cout << "[INFO]: Finished processing frame." << std::endl << std::endl;

		while (std::getline(ss, frame_name, '/'));
		logger_->logBoundingBoxFrame(frame_name, bounding_box_type_);
	}
}

void
DepthClustering::finish()
{
	logger_->writeBoundingBoxLog(log_path_, log_file_name_);
}
