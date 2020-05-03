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
using depth_clustering::Radians;
using depth_clustering::RichPoint;

DepthClustering::DepthClustering()
{
	size_cluster_min_ = 10;
	size_cluster_max_ = 10000;
	size_smooth_window_ = 5;
	angle_clustering_ = 10_deg;
	angle_ground_removal_ = 9_deg;
	log_apollo_ = false;
	log_data_ = true;
}

DepthClustering::DepthClustering(int size_cluster_min, int size_cluster_max, int size_smooth_window,
		float angle_clustering, float angle_ground_removal, bool log_apollo) :
		size_cluster_min_(size_cluster_min), size_cluster_max_(size_cluster_max), size_smooth_window_(
				size_smooth_window), log_apollo_(log_apollo)
{
	angle_clustering_ = Radians
	{ Radians::IsRadians
	{ }, static_cast<float>(angle_clustering * M_PI / 180.0) };
	angle_ground_removal_ = Radians
	{ Radians::IsRadians
	{ }, static_cast<float>(angle_ground_removal * M_PI / 180.0) };
	log_data_ = false;
}

void
DepthClustering::init_apollo_box()
{
	projection_parameter_ = ProjectionParams::APOLLO();
	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*projection_parameter_,
			angle_ground_removal_, size_smooth_window_);
	clusterer_ = std::make_shared<ImageBasedClusterer<LinearImageLabeler<>>>(angle_clustering_,
			size_cluster_min_, size_cluster_max_);
	object_painter_.reset(new ObjectPainter
	{ ObjectPainter::OutlineType::kBox, &output_box_frame_, nullptr, log_apollo_ });

	clusterer_->SetDiffType(DiffFactory::DiffType::ANGLES);

	depth_ground_remover_->AddClient(clusterer_.get());
	clusterer_->AddClient(object_painter_.get());
}

void
DepthClustering::init_apollo_polygon()
{
	projection_parameter_ = ProjectionParams::APOLLO();
	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*projection_parameter_,
			angle_ground_removal_, size_smooth_window_);
	clusterer_ = std::make_shared<ImageBasedClusterer<LinearImageLabeler<>>>(angle_clustering_,
			size_cluster_min_, size_cluster_max_);
	object_painter_.reset(new ObjectPainter
	{ ObjectPainter::OutlineType::kPolygon3d, nullptr, &output_polygon_frame_, log_apollo_ });

	clusterer_->SetDiffType(DiffFactory::DiffType::ANGLES);

	depth_ground_remover_->AddClient(clusterer_.get());
	clusterer_->AddClient(object_painter_.get());
}

void
DepthClustering::init_data_box(const std::string& data_folder)
{
	folder_reader_data_ = std::make_shared<FolderReader>(data_folder, ".png",
			FolderReader::Order::SORTED);
	folder_reader_config_ = std::make_shared<FolderReader>(data_folder, "img.cfg");
	projection_parameter_ = ProjectionParams::HDL_64();
	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*projection_parameter_,
			angle_ground_removal_, size_smooth_window_);
	clusterer_ = std::make_shared<ImageBasedClusterer<LinearImageLabeler<>>>(angle_clustering_,
			size_cluster_min_, size_cluster_max_);
	object_painter_.reset(new ObjectPainter
	{ ObjectPainter::OutlineType::kBox, &output_box_frame_, nullptr, log_data_ });

	clusterer_->SetDiffType(DiffFactory::DiffType::ANGLES_PRECOMPUTED);

	depth_ground_remover_->AddClient(clusterer_.get());
	clusterer_->AddClient(object_painter_.get());
}

void
DepthClustering::init_data_polygon(const std::string& data_folder)
{
	folder_reader_data_ = std::make_shared<FolderReader>(data_folder, ".png",
			FolderReader::Order::SORTED);
	folder_reader_config_ = std::make_shared<FolderReader>(data_folder, "img.cfg");
	projection_parameter_ = ProjectionParams::HDL_64();
	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*projection_parameter_,
			angle_ground_removal_, size_smooth_window_);
	clusterer_ = std::make_shared<ImageBasedClusterer<LinearImageLabeler<>>>(angle_clustering_,
			size_cluster_min_, size_cluster_max_);
	object_painter_.reset(new ObjectPainter
	{ ObjectPainter::OutlineType::kPolygon3d, nullptr, &output_polygon_frame_, log_data_ });

	clusterer_->SetDiffType(DiffFactory::DiffType::ANGLES_PRECOMPUTED);

	depth_ground_remover_->AddClient(clusterer_.get());
	clusterer_->AddClient(object_painter_.get());
}

std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>
DepthClustering::process_apollo_box(const std::string& frame_name,
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

	output_box_frame_.clear();

	depth_ground_remover_->OnNewObjectReceived(std::make_pair(frame_name, *cloud), 0);

	return output_box_frame_;
}

std::vector<std::pair<ObjectPainter::AlignedEigenVectors, float>>
DepthClustering::process_apollo_polygon(const std::string& frame_name,
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

	output_polygon_frame_.clear();

	depth_ground_remover_->OnNewObjectReceived(std::make_pair(frame_name, *cloud), 0);

	return output_polygon_frame_;
}

std::vector<ObjectPainter::OutputBoxFrame>
DepthClustering::process_data_box()
{
	for (const auto &path : folder_reader_data_->GetAllFilePaths())
	{
		auto depth_image = MatFromDepthPng(path);
		auto cloud = Cloud::FromImage(depth_image, *projection_parameter_);

		output_box_frame_.clear();

		depth_ground_remover_->OnNewObjectReceived(std::make_pair(path, *cloud), 0);

		outputs_box_frame_.push_back(output_box_frame_);
	}

	return outputs_box_frame_;
}

std::vector<ObjectPainter::OutputPolygonFrame>
DepthClustering::process_data_polygon()
{
	for (const auto &path : folder_reader_data_->GetAllFilePaths())
	{
		auto depth_image = MatFromDepthPng(path);
		auto cloud = Cloud::FromImage(depth_image, *projection_parameter_);

		output_polygon_frame_.clear();

		depth_ground_remover_->OnNewObjectReceived(std::make_pair(path, *cloud), 0);

		outputs_polygon_frame_.push_back(output_polygon_frame_);
	}

	return outputs_polygon_frame_;
}

void
DepthClustering::finish()
{
	object_painter_->writeLog();
}
