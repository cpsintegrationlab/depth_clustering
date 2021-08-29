// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#ifndef SRC_QT_PCL_FOLDER_PLAYER_H_
#define SRC_QT_PCL_FOLDER_PLAYER_H_

#include <memory>
#include <opencv2/opencv.hpp>
#include <QGraphicsScene>
#include <QKeyEvent>
#include <QWidget>
#include <string>
#include <vector>

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
	OnNewObjectReceived(const cv::Mat& labels, int client_id = 0) override;
	virtual
	~Visualization();

protected:
	void
	keyPressEvent(QKeyEvent* event) override;

	bool
	eventFilter(QObject* obj, QEvent* event) override;

private slots:
	void
	onOpenFolderToRead();
	void
	onPlayAllClouds();
	void
	onSegmentationParamUpdate();
	void
	onSliderMovedTo(int cloud_number);

private:
	// we cannot allocate anything now as this is an incomplete type
	std::unique_ptr<Ui::Visualization> ui;

	std::unique_ptr<QGraphicsScene> _scene = nullptr;
	std::unique_ptr<QGraphicsScene> _scene_labels = nullptr;
	std::unique_ptr<depth_clustering::ProjectionParams> _proj_params = nullptr;

	std::unique_ptr<depth_clustering::ImageBasedClusterer<depth_clustering::LinearImageLabeler<>>> _clusterer =
			nullptr;
	std::unique_ptr<depth_clustering::DepthGroundRemover> _ground_rem = nullptr;

	std::unique_ptr<depth_clustering::BoundingBox> _bounding_box = nullptr;

	cv::Mat _current_full_depth_image;
	depth_clustering::Cloud::Ptr _cloud;
	std::vector<std::string> _file_names;

	int _current_coloring_mode = 0;

	std::vector<std::string> _current_object_labels;

	Viewer *_viewer = nullptr;
};

#endif  // SRC_QT_PCL_FOLDER_PLAYER_H_
