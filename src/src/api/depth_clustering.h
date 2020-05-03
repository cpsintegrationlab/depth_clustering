/*
 * depth_clustering.h
 *
 *  Created on: Apr 27, 2020
 *      Author: simonyu
 */

#ifndef SRC_API_DEPTH_CLUSTERING_H_
#define SRC_API_DEPTH_CLUSTERING_H_

#include <memory>
#include <thread>
#include <string>

#include "clusterers/image_based_clusterer.h"
#include "ground_removal/depth_ground_remover.h"
#include "image_labelers/linear_image_labeler.h"
#include "projections/projection_params.h"
#include "qt/drawables/object_painter.h"
#include "utils/folder_reader.h"
#include "utils/radians.h"

using depth_clustering::DepthGroundRemover;
using depth_clustering::FolderReader;
using depth_clustering::ImageBasedClusterer;
using depth_clustering::LinearImageLabeler;
using depth_clustering::ObjectPainter;
using depth_clustering::ProjectionParams;
using depth_clustering::Radians;

class QApplication;
class Viewer;

class DepthClustering
{
public:

	DepthClustering();

	void
	init_apollo_box();

	void
	init_apollo_polygon();

	void
	init_data_box(const std::string& data_folder);

	void
	init_data_polygon(const std::string& data_folder);

	std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>
	process_apollo_box(const std::string& frame_name,
			const std::vector<Eigen::Vector3f>& point_cloud);

	std::vector<std::pair<ObjectPainter::AlignedEigenVectors, float>>
	process_apollo_polygon(const std::string& frame_name,
			const std::vector<Eigen::Vector3f>& point_cloud);

	std::vector<ObjectPainter::OutputBoxFrame>
	process_data_box();

	std::vector<ObjectPainter::OutputPolygonFrame>
	process_data_polygon();

	void
	finish();

private:

	Radians angle_clustering_;
	Radians angle_ground_removal_;
	int size_cluster_min_;
	int size_cluster_max_;
	int size_smooth_window_;
	bool log_apollo_;
	bool log_data_;

	std::shared_ptr<FolderReader> folder_reader_data_;
	std::shared_ptr<FolderReader> folder_reader_config_;
	std::unique_ptr<ProjectionParams> projection_parameter_;
	std::shared_ptr<DepthGroundRemover> depth_ground_remover_;
	std::shared_ptr<ImageBasedClusterer<LinearImageLabeler<>>> clusterer_;
	std::unique_ptr<ObjectPainter> object_painter_;

	ObjectPainter::OutputBoxFrame output_box_frame_;
	ObjectPainter::OutputPolygonFrame output_polygon_frame_;
	std::vector<ObjectPainter::OutputBoxFrame> outputs_box_frame_;
	std::vector<ObjectPainter::OutputPolygonFrame> outputs_polygon_frame_;
};

#endif /* SRC_API_DEPTH_CLUSTERING_H_ */
