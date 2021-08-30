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

class Visualization: public QWidget, public depth_clustering::AbstractClient<cv::Mat>
{
Q_OBJECT

public:

	explicit
	Visualization(QWidget* parent = 0);

	void
	OnNewObjectReceived(const cv::Mat& image_segmentation, int client_id = 0) override;

	virtual
	~Visualization();

protected:

	bool
	eventFilter(QObject* object, QEvent* event) override;

	void
	keyPressEvent(QKeyEvent* event) override;

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

private:

	void
	updateViewerPointCloud();

	void
	updateViewerImage();

	std::unique_ptr<Ui::Visualization> ui;
	std::unique_ptr<QGraphicsScene> scene_difference_ = nullptr;
	std::unique_ptr<QGraphicsScene> scene_segmentation_ = nullptr;
	std::unique_ptr<QGraphicsScene> scene_depth_ = nullptr;
	std::unique_ptr<DepthClustering> depth_clustering_ = nullptr;

	std::string dataset_path_;
	volatile bool play_;
};

#endif  // SRC_VISUALIZATION_VISUALIZATION_H_
