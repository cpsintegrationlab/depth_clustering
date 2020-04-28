/*
 * depth_clustering.cpp
 *
 *  Created on: Apr 27, 2020
 *      Author: simonyu
 */

#include <QApplication>

#include "api/depth_clustering.h"
#include "clusterers/image_based_clusterer.h"
#include "ground_removal/depth_ground_remover.h"
#include "image_labelers/diff_helpers/diff_factory.h"
#include "image_labelers/linear_image_labeler.h"
#include "projections/projection_params.h"
#include "qt/drawables/drawable_cloud.h"
#include "qt/drawables/object_painter.h"
#include "qt/viewer/viewer.h"
#include "utils/cloud.h"
#include "utils/folder_reader.h"
#include "utils/velodyne_utils.h"

using depth_clustering::Cloud;
using depth_clustering::DepthGroundRemover;
using depth_clustering::DiffFactory;
using depth_clustering::FolderReader;
using depth_clustering::ImageBasedClusterer;
using depth_clustering::LinearImageLabeler;
using depth_clustering::MatFromDepthPng;
using depth_clustering::ObjectPainter;
using depth_clustering::ProjectionParams;

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
DepthClustering::process(const std::string& data_folder)
{
	auto folder_reader_data = FolderReader(data_folder, ".png", FolderReader::Order::SORTED);
	auto folder_reader_config = FolderReader(data_folder, "img.cfg");
	auto projection_parameter = ProjectionParams::FromConfigFile(
			folder_reader_config.GetNextFilePath());
	auto depth_ground_remover = DepthGroundRemover(*projection_parameter, angle_ground_removal_,
			size_smooth_window_);

	ImageBasedClusterer<LinearImageLabeler<>> clusterer(angle_clustering_, size_cluster_min_,
			size_cluster_max_);

	clusterer.SetDiffType(DiffFactory::DiffType::ANGLES);

	ObjectPainter object_painter
	{ viewer_.get(), ObjectPainter::OutlineType::kPolygon3d };

	depth_ground_remover.AddClient(&clusterer);
	clusterer.AddClient(&object_painter);

	for (const auto &path : folder_reader_data.GetAllFilePaths())
	{
		auto depth_image = MatFromDepthPng(path);
		auto cloud = Cloud::FromImage(depth_image, *projection_parameter);
		viewer_->Clear();
		viewer_->AddDrawable(DrawableCloud::FromCloud(cloud));
		depth_ground_remover.OnNewObjectReceived(std::make_pair(path, *cloud), 0);
	}
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
