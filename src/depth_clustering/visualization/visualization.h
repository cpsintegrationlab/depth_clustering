/*
 * visualization.h
 *
 *  Created on: Aug 29, 2021
 *      Author: simonyu
 */

#ifndef SRC_VISUALIZATION_VISUALIZATION_H_
#define SRC_VISUALIZATION_VISUALIZATION_H_

#include <memory>
#include <opencv2/opencv.hpp>
#include <QGraphicsScene>
#include <QKeyEvent>
#include <QWidget>
#include <string>
#include <vector>

#include "api/api.h"
#include "clusterers/image_based_clusterer.h"
#include "communication/abstract_client.h"
#include "ground_removal/depth_ground_remover.h"
#include "post_processing/bounding_box.h"
#include "projections/cloud_projection.h"
#include "projections/spherical_projection.h"
#include "utils/cloud.h"
#include "visualization/viewer/viewer.h"

namespace Ui
{
class Visualization;
}

class Visualization: public QWidget,
		public depth_clustering::AbstractClient<cv::Mat>,
		public depth_clustering::AbstractClient<Cloud>
{
Q_OBJECT

public:

	explicit
	Visualization(QWidget* parent = 0);

	void
	OnNewObjectReceived(const cv::Mat& image_segmentation, int client_id = 0) override;

	void
	OnNewObjectReceived(const Cloud& cloud_no_ground, int client_id = 0) override;

	virtual
	~Visualization();

protected:

	bool
	eventFilter(QObject* object, QEvent* event) override;

	void
	keyPressEvent(QKeyEvent* event) override;

	void
	showEvent(QShowEvent* event) override;

	void
	resizeEvent(QResizeEvent* event) override;

private slots:

	void
	onOpen();

	void
	onPlay();

	void
	onPause();

	void
	onStop();

	void
	onSliderMovedTo(int frame_number);

	void
	onParameterUpdated();

	void
	onLoadGlobalConfiguration();

	void
	onSplitterViewerMoved();

	void
	onLayerPointCloudUpdated();

	void
	onLayerImageUpdated();

	void
	onDifferenceTypeUpdated();

	void
	onNextPage();

	void
	onLastPage();

private:

	void
	openDataset(const std::string& dataset_path, const std::string& global_config_path = "");

	std::pair<Cloud::ConstPtr, Cloud::ConstPtr>
	getGroundPointCloudPair();

	void
	updateViewerPointCloud();

	void
	updateViewerImageScene(const std::string& frame_path_name_camera);

	void
	updateViewerImage();

	void
	resetViewer();

	void
	resetUI();

	void
	initializeUI();

	std::unique_ptr<Ui::Visualization> ui;

	std::shared_ptr<DepthClustering> depth_clustering_ = nullptr;
	std::shared_ptr<DepthClustering> depth_clustering_first_return_ = nullptr;
	std::shared_ptr<DepthClustering> depth_clustering_second_return_ = nullptr;
	std::shared_ptr<BoundingBox::Frame<BoundingBox::Flat>> ground_truth_frame_flat_ = nullptr;
	std::unique_ptr<QGraphicsScene> scene_empty_ = nullptr;
	std::unique_ptr<QGraphicsScene> scene_camera_ = nullptr;
	std::unique_ptr<QGraphicsScene> scene_difference_ = nullptr;
	std::unique_ptr<QGraphicsScene> scene_segmentation_ = nullptr;
	std::unique_ptr<QGraphicsScene> scene_range_ = nullptr;
	std::unique_ptr<QGraphicsScene> scene_intensity_ = nullptr;
	std::unique_ptr<QGraphicsScene> scene_elongation_ = nullptr;

	std::string dataset_path_;
	std::string global_config_path_;
	volatile bool play_;
	bool shown_;
	bool viewer_image_camera_visible_;
	bool show_bounding_box_;
	int viewer_point_cloud_layer_index_;
	int viewer_image_layer_index_top_;
	int viewer_image_layer_index_middle_;
	int viewer_image_layer_index_bottom_;

	cv::Mat image_range_;
	cv::Mat image_range_no_ground_;
	mutable std::mutex image_range_mutex_;
};

#endif  // SRC_VISUALIZATION_VISUALIZATION_H_
