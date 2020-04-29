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

#include "qt/drawables/object_painter.h"
#include "utils/radians.h"

using depth_clustering::ObjectPainter;
using depth_clustering::Radians;

class QApplication;
class Viewer;

class DepthClustering
{
public:

	DepthClustering();

	~DepthClustering();

	void
	process_frame_box();

	void
	process_frame_polygon();

	std::queue<ObjectPainter::OutputBoxFrame>
	process_data_box(const std::string& data_folder);

	std::queue<ObjectPainter::OutputPolygonFrame>
	process_data_polygon(const std::string& data_folder);

private:

	void
	viewerThread();

	Radians angle_clustering_;
	Radians angle_ground_removal_;
	int size_cluster_min_;
	int size_cluster_max_;
	int size_smooth_window_;

	std::shared_ptr<Viewer> viewer_;
	std::thread viewer_thread_;
};

#endif /* SRC_API_DEPTH_CLUSTERING_H_ */
