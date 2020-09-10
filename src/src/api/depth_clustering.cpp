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
	data_type_ = ".tiff";
	bounding_box_type_ = BoundingBox::Type::Cube;
	size_cluster_min_ = 10;
	size_cluster_max_ = 20000;
	size_smooth_window_ = 5;
	angle_clustering_ = 10_deg;
	angle_ground_removal_ = 9_deg;
	log_apollo_ = false;
	log_data_ = true;
}

DepthClustering::DepthClustering(std::string data_type, BoundingBox::Type bounding_box_type,
		int size_cluster_min, int size_cluster_max, int size_smooth_window,
		float angle_clustering, float angle_ground_removal, bool log_apollo, bool log_data) :
		data_type_(data_type), bounding_box_type_(bounding_box_type), size_cluster_min_(size_cluster_min), size_cluster_max_(
				size_cluster_max), size_smooth_window_(size_smooth_window), log_apollo_(log_apollo), log_data_(log_data)
{
	angle_clustering_ = Radians
	{ Radians::IsRadians
	{ }, static_cast<float>(angle_clustering * M_PI / 180.0) };
	angle_ground_removal_ = Radians
	{ Radians::IsRadians
	{ }, static_cast<float>(angle_ground_removal * M_PI / 180.0) };
}

bool
DepthClustering::init_apollo(const BoundingBox::Type& bounding_box_type)
{
	bounding_box_type_ = bounding_box_type;
	projection_parameter_ = ProjectionParams::APOLLO();
	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*projection_parameter_,
			angle_ground_removal_, size_smooth_window_);
	clusterer_ = std::make_shared<ImageBasedClusterer<LinearImageLabeler<>>>(angle_clustering_,
			size_cluster_min_, size_cluster_max_);

	clusterer_->SetDiffType(DiffFactory::DiffType::ANGLES);

	resetBoundingBox(log_apollo_);

	depth_ground_remover_->AddClient(clusterer_.get());
	clusterer_->AddClient(bounding_box_.get());

	return true;
}

bool
DepthClustering::init_data(const std::string& data_folder, const std::string& data_type,
		const BoundingBox::Type& bounding_box_type)
{
	data_type_ = data_type;
	bounding_box_type_ = bounding_box_type;
	folder_reader_data_ = std::make_shared<FolderReader>(data_folder, data_type_,
			FolderReader::Order::SORTED);
	folder_reader_config_ = std::make_shared<FolderReader>(data_folder, "img.cfg");
	clusterer_ = std::make_shared<ImageBasedClusterer<LinearImageLabeler<>>>(angle_clustering_,
			size_cluster_min_, size_cluster_max_);

	clusterer_->SetDiffType(DiffFactory::DiffType::ANGLES);

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

	resetBoundingBox(log_data_);

	depth_ground_remover_->AddClient(clusterer_.get());
	clusterer_->AddClient(bounding_box_.get());

	return true;
}

void
DepthClustering::process_apollo(const std::string& frame_name,
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

	clearOutputFrame();

	depth_ground_remover_->OnNewObjectReceived(std::make_pair(frame_name, *cloud), 0);
}

void
DepthClustering::process_data()
{
	for (const auto &path : folder_reader_data_->GetAllFilePaths())
	{
		cv::Mat depth_image;

		if (data_type_ == ".png")
		{
			depth_image = MatFromDepthPng(path);
		}
		else if (data_type_ == ".tiff")
		{
			depth_image = MatFromDepthTiff(path);
		}
		else
		{
			std::cout << "Unknown data type. Skip." << std::endl;
			return;
		}

		auto cloud = Cloud::FromImage(depth_image, *projection_parameter_);

		clearOutputFrame();
		std::cout << "[INFO]: Started processing frame." << std::endl;

		depth_ground_remover_->OnNewObjectReceived(std::make_pair(path, *cloud), 0);

		storeOutputFrame();
		std::cout << "[INFO]: Finished processing frame." << std::endl << std::endl;
	}
}

void
DepthClustering::finish()
{
	bounding_box_->writeLog();
}

void
DepthClustering::resetBoundingBox(bool& log)
{
	switch (bounding_box_type_)
	{
	case BoundingBox::Type::Cube:
	{
		bounding_box_.reset(new BoundingBox
			{ bounding_box_type_, &bounding_box_frame_cube_, nullptr, log });

		break;
	}
	case BoundingBox::Type::Polygon:
	{
		bounding_box_.reset(new BoundingBox
			{ bounding_box_type_, nullptr, &bounding_box_frame_polygon_, log });

		break;
	}
	default:
	{
		bounding_box_.reset(new BoundingBox
			{ bounding_box_type_, &bounding_box_frame_cube_, nullptr, log });

		break;
	}
	}
}

void
DepthClustering::clearOutputFrame()
{
	switch (bounding_box_type_)
	{
	case BoundingBox::Type::Cube:
	{
		bounding_box_frame_cube_.clear();
		break;
	}
	case BoundingBox::Type::Polygon:
	{
		bounding_box_frame_polygon_.clear();
		break;
	}
	default:
	{
		bounding_box_frame_cube_.clear();
		break;
	}
	}
}

void
DepthClustering::storeOutputFrame()
{
	switch (bounding_box_type_)
	{
	case BoundingBox::Type::Cube:
	{
		bounding_box_frames_cube_.push_back(bounding_box_frame_cube_);
		break;
	}
	case BoundingBox::Type::Polygon:
	{
		bounding_box_frames_polygon_.push_back(bounding_box_frame_polygon_);
		break;
	}
	default:
	{
		bounding_box_frames_cube_.push_back(bounding_box_frame_cube_);
		break;
	}
	}
}
