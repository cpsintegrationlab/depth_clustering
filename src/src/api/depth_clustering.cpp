/*
 * depth_clustering.cpp
 *
 *  Created on: Apr 27, 2020
 *      Author: simonyu
 */

#include <QApplication>

#include "api/depth_clustering.h"
#include "image_labelers/diff_helpers/diff_factory.h"
#include "qt/drawables/drawable_cloud.h"
#include "qt/viewer/viewer.h"
#include "utils/cloud.h"
#include "utils/rich_point.h"
#include "utils/velodyne_utils.h"

using depth_clustering::Cloud;
using depth_clustering::DiffFactory;
using depth_clustering::MatFromDepthPng;

using depth_clustering::RichPoint;

DepthClustering::DepthClustering()
{
	size_cluster_min_ = 20;
	size_cluster_max_ = 100000;
	size_smooth_window_ = 5;
	angle_clustering_ = 10_deg;
	angle_ground_removal_ = 9_deg;

	viewer_thread_ = std::thread(&DepthClustering::viewerThread, this);

	std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

DepthClustering::~DepthClustering()
{
	viewer_thread_.join();
}

void
DepthClustering::init_apollo_box()
{
	projection_parameter_ = ProjectionParams::HDL_32();
	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*projection_parameter_,
			angle_ground_removal_, size_smooth_window_);
	clusterer_ = std::make_shared<ImageBasedClusterer<LinearImageLabeler<>>>(angle_clustering_,
			size_cluster_min_, size_cluster_max_);
	object_painter_.reset(new ObjectPainter
	{ viewer_.get(), ObjectPainter::OutlineType::kBox, &output_box_frame_, nullptr, false });

	clusterer_->SetDiffType(DiffFactory::DiffType::ANGLES);

	depth_ground_remover_->AddClient(clusterer_.get());
	clusterer_->AddClient(object_painter_.get());
}

void
DepthClustering::init_apollo_polygon()
{
	projection_parameter_ = ProjectionParams::HDL_32();
	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*projection_parameter_,
			angle_ground_removal_, size_smooth_window_);
	clusterer_ = std::make_shared<ImageBasedClusterer<LinearImageLabeler<>>>(angle_clustering_,
			size_cluster_min_, size_cluster_max_);
	object_painter_.reset(
			new ObjectPainter
			{ viewer_.get(), ObjectPainter::OutlineType::kPolygon3d, nullptr,
					&output_polygon_frame_, false });

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
	projection_parameter_ = ProjectionParams::FromConfigFile(
			folder_reader_config_->GetNextFilePath());
	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*projection_parameter_,
			angle_ground_removal_, size_smooth_window_);
	clusterer_ = std::make_shared<ImageBasedClusterer<LinearImageLabeler<>>>(angle_clustering_,
			size_cluster_min_, size_cluster_max_);
	object_painter_.reset(new ObjectPainter
	{ viewer_.get(), ObjectPainter::OutlineType::kBox, &output_box_frame_, nullptr, false });

	clusterer_->SetDiffType(DiffFactory::DiffType::ANGLES);

	depth_ground_remover_->AddClient(clusterer_.get());
	clusterer_->AddClient(object_painter_.get());
}

void
DepthClustering::init_data_polygon(const std::string& data_folder)
{
	folder_reader_data_ = std::make_shared<FolderReader>(data_folder, ".png",
			FolderReader::Order::SORTED);
	folder_reader_config_ = std::make_shared<FolderReader>(data_folder, "img.cfg");
	projection_parameter_ = ProjectionParams::FromConfigFile(
			folder_reader_config_->GetNextFilePath());
	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*projection_parameter_,
			angle_ground_removal_, size_smooth_window_);
	clusterer_ = std::make_shared<ImageBasedClusterer<LinearImageLabeler<>>>(angle_clustering_,
			size_cluster_min_, size_cluster_max_);
	object_painter_.reset(
			new ObjectPainter
			{ viewer_.get(), ObjectPainter::OutlineType::kPolygon3d, nullptr,
					&output_polygon_frame_, false });

	clusterer_->SetDiffType(DiffFactory::DiffType::ANGLES);

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

	viewer_->Clear();
	viewer_->AddDrawable(DrawableCloud::FromCloud(cloud));

	output_box_frame_.clear();

	depth_ground_remover_->OnNewObjectReceived(std::make_pair(frame_name, *cloud), 0);

	return output_box_frame_;
}

std::vector<std::pair<DrawablePolygon3d::AlignedEigenVectors, float>>
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

	viewer_->Clear();
	viewer_->AddDrawable(DrawableCloud::FromCloud(cloud));

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

		viewer_->Clear();
		viewer_->AddDrawable(DrawableCloud::FromCloud(cloud));

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

		viewer_->Clear();
		viewer_->AddDrawable(DrawableCloud::FromCloud(cloud));

		output_polygon_frame_.clear();

		depth_ground_remover_->OnNewObjectReceived(std::make_pair(path, *cloud), 0);

		outputs_polygon_frame_.push_back(output_polygon_frame_);
	}

	return outputs_polygon_frame_;
}

void
DepthClustering::viewerThread()
{
	char arg0[] = "depth_clustering";
	char arg1[] = "arg1";
	char arg2[] = "arg2";
	char *argv[] =
	{ &arg0[0], &arg1[0], &arg2[0], NULL };
	int argc = static_cast<int>(sizeof(argv) / sizeof(argv[0]) - 1);
	QApplication q_application(argc, argv);

	viewer_ = std::make_shared<Viewer>();

	if (viewer_)
	{
		viewer_->show();
	}

	q_application.exec();
}
