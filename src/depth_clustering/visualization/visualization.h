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
#include "visualization/viewer/viewer.h"
#include "visualization/visualization_layout.h"

using depth_clustering::BoundingBox;
using depth_clustering::DepthClustering;
using depth_clustering::AbstractClient;

namespace Ui
{
class Visualization;
}

namespace visualization
{
class Visualization: public QWidget, public AbstractClient<cv::Mat>, public AbstractClient<
		std::pair<cv::Mat, cv::Mat>>
{
Q_OBJECT

public:

	explicit
	Visualization(QWidget* parent = 0);

	void
	OnNewObjectReceived(const cv::Mat& image_segmentation, int id = 0) override;

	void
	OnNewObjectReceived(const std::pair<cv::Mat, cv::Mat>& images, int id = 0) override;

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
	onLoadLayoutConfiguration();

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
	openDataset(const std::string& dataset_path, const std::string& file_path_name_config_global =
			"");

	void
	updateViewerPointCloud();

	void
	updateViewerImageScene(const std::string& frame_path_name_camera = "");

	void
	updateViewerImage();

	void
	resetViewer();

	void
	resetUI();

	void
	initializeUI();

	void
	connectSignals();

	void
	disconnectSignals();

	std::unique_ptr<Ui::Visualization> ui;

	std::shared_ptr<DepthClustering> depth_clustering_ = nullptr;
	std::shared_ptr<DepthClustering> depth_clustering_first_return_ = nullptr;
	std::shared_ptr<DepthClustering> depth_clustering_second_return_ = nullptr;
	std::shared_ptr<BoundingBox::Frame<BoundingBox::Flat>> ground_truth_frame_flat_ = nullptr;
	std::shared_ptr<BoundingBox::Frame<BoundingBox::Cube>> ground_truth_frame_cube_ = nullptr;
	std::unique_ptr<QGraphicsScene> scene_empty_ = nullptr;
	std::unique_ptr<QGraphicsScene> scene_camera_ = nullptr;
	std::unique_ptr<QGraphicsScene> scene_difference_ = nullptr;
	std::unique_ptr<QGraphicsScene> scene_segmentation_ = nullptr;
	std::unique_ptr<QGraphicsScene> scene_range_ = nullptr;
	std::unique_ptr<QGraphicsScene> scene_intensity_ = nullptr;
	std::unique_ptr<QGraphicsScene> scene_elongation_ = nullptr;

	VisualizationLayout layout_;
	std::string dataset_path_;
	std::string file_path_name_config_global_;
	volatile bool play_;
	bool initialized_;

	cv::Mat image_range_ground_;
	cv::Mat image_range_no_ground_;
};
} // namespace visualization

#endif  // SRC_VISUALIZATION_VISUALIZATION_H_
