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
	outline_type_ = ObjectPainter::OutlineType::kBox;
	size_cluster_min_ = 10;
	size_cluster_max_ = 20000;
	size_smooth_window_ = 5;
	angle_clustering_ = 10_deg;
	angle_ground_removal_ = 9_deg;
	log_apollo_ = false;
	log_data_ = true;
}

DepthClustering::DepthClustering(std::string data_type, ObjectPainter::OutlineType outline_type,
		int size_cluster_min, int size_cluster_max, int size_smooth_window,
		float angle_clustering, float angle_ground_removal, bool log_apollo, bool log_data) :
		data_type_(data_type), outline_type_(outline_type), size_cluster_min_(size_cluster_min), size_cluster_max_(
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
DepthClustering::init_apollo(const ObjectPainter::OutlineType& outline_type)
{
	outline_type_ = outline_type;
	projection_parameter_ = ProjectionParams::APOLLO();
	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*projection_parameter_,
			angle_ground_removal_, size_smooth_window_);
	clusterer_ = std::make_shared<ImageBasedClusterer<LinearImageLabeler<>>>(angle_clustering_,
			size_cluster_min_, size_cluster_max_);

	clusterer_->SetDiffType(DiffFactory::DiffType::ANGLES);

	resetObjectPainter(log_apollo_);

	depth_ground_remover_->AddClient(clusterer_.get());
	clusterer_->AddClient(object_painter_.get());

	return true;
}

bool
DepthClustering::init_data(const std::string& data_folder, const std::string& data_type,
		const ObjectPainter::OutlineType& outline_type)
{
	data_type_ = data_type;
	outline_type_ = outline_type;
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

	resetObjectPainter(log_data_);

	depth_ground_remover_->AddClient(clusterer_.get());
	clusterer_->AddClient(object_painter_.get());

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

		depth_ground_remover_->OnNewObjectReceived(std::make_pair(path, *cloud), 0);

		storeOutputFrame();
	}
}

std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>
DepthClustering::get_output_apollo_box() const
{
	return output_box_frame_;
}

std::vector<std::pair<ObjectPainter::AlignedEigenVectors, float>>
DepthClustering::get_output_apollo_polygon() const
{
	return output_polygon_frame_;
}

std::vector<ObjectPainter::OutputBoxFrame>
DepthClustering::get_output_data_box() const
{
	return outputs_box_frame_;
}

std::vector<ObjectPainter::OutputPolygonFrame>
DepthClustering::get_output_data_polygon() const
{
	return outputs_polygon_frame_;
}

void
DepthClustering::finish()
{
	object_painter_->writeLog();
}

void
DepthClustering::resetObjectPainter(bool& log)
{
	switch (outline_type_)
	{
	case ObjectPainter::OutlineType::kBox:
	{
		object_painter_.reset(new ObjectPainter
			{ outline_type_, &output_box_frame_, nullptr, log });

		break;
	}
	case ObjectPainter::OutlineType::kPolygon3d:
	{
		object_painter_.reset(new ObjectPainter
			{ outline_type_, nullptr, &output_polygon_frame_, log });

		break;
	}
	default:
	{
		object_painter_.reset(new ObjectPainter
			{ outline_type_, &output_box_frame_, nullptr, log });

		break;
	}
	}
}

void
DepthClustering::clearOutputFrame()
{
	switch (outline_type_)
	{
	case ObjectPainter::OutlineType::kBox:
	{
		output_box_frame_.clear();
		break;
	}
	case ObjectPainter::OutlineType::kPolygon3d:
	{
		output_polygon_frame_.clear();
		break;
	}
	default:
	{
		output_box_frame_.clear();
		break;
	}
	}
}

void
DepthClustering::storeOutputFrame()
{
	switch (outline_type_)
	{
	case ObjectPainter::OutlineType::kBox:
	{
		outputs_box_frame_.push_back(output_box_frame_);
		break;
	}
	case ObjectPainter::OutlineType::kPolygon3d:
	{
		outputs_polygon_frame_.push_back(output_polygon_frame_);
		break;
	}
	default:
	{
		outputs_box_frame_.push_back(output_box_frame_);
		break;
	}
	}
}
