/*
 * visualization.cpp
 *
 *  Created on: Aug 29, 2021
 *      Author: simonyu
 */

#include <QColor>
#include <QDebug>
#include <QFileDialog>
#include <QImage>
#include <QPixmap>
#include <QString>
#include <QUuid>
#include <vector>

#include "visualization/drawables/drawable_cloud.h"
#include "visualization/drawables/drawable_cube.h"
#include "visualization/drawables/drawable_polygon3d.h"
#include "visualization/utils/utils.h"
#include "visualization/ui_visualization.h"
#include "visualization/visualization.h"

using depth_clustering::AbstractImageLabeler;
using depth_clustering::Cloud;
using depth_clustering::DepthClusteringParameter;
using depth_clustering::DiffFactory;
using depth_clustering::DrawableCube;
using depth_clustering::DrawablePolygon3d;
using depth_clustering::FolderReader;
using depth_clustering::ProjectionParams;
using depth_clustering::Radians;
using depth_clustering::Score;
using depth_clustering::time_utils::Timer;

namespace visualization
{
Visualization::Visualization(QWidget* parent) :
		QWidget(parent), ui(new Ui::Visualization), play_(false), initialized_(false)
{
	ui->setupUi(this);

	ui->frame_controls->setFixedHeight(ui->frame_controls->minimumHeight());
	ui->frame_settings->setFixedHeight(ui->frame_settings->minimumHeight());
	ui->frame_viewer_image->setFixedHeight(ui->frame_viewer_image->minimumHeight());

	ui->viewer_image_camera->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	ui->viewer_image_camera->setCacheMode(QGraphicsView::CacheBackground);
	ui->viewer_image_camera->setRenderHints(
			QPainter::SmoothPixmapTransform | QPainter::Antialiasing
					| QPainter::HighQualityAntialiasing);
	ui->viewer_image_left->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	ui->viewer_image_left->setCacheMode(QGraphicsView::CacheBackground);
	ui->viewer_image_left->setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
	ui->viewer_image_middle->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	ui->viewer_image_middle->setCacheMode(QGraphicsView::CacheBackground);
	ui->viewer_image_middle->setRenderHints(
			QPainter::SmoothPixmapTransform | QPainter::Antialiasing
					| QPainter::HighQualityAntialiasing);
	ui->viewer_image_right->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	ui->viewer_image_right->setCacheMode(QGraphicsView::CacheBackground);
	ui->viewer_image_right->setRenderHints(
			QPainter::SmoothPixmapTransform | QPainter::Antialiasing
					| QPainter::HighQualityAntialiasing);
	ui->viewer_point_cloud->installEventFilter(this);
	ui->viewer_point_cloud->setAutoFillBackground(true);

	setWindowTitle(QCoreApplication::arguments().at(0));

	connect(ui->button_open, SIGNAL(released()), this, SLOT(onOpen()));
	connect(ui->button_page_next_page_1, SIGNAL(released()), this, SLOT(onNextPage()));
	connect(ui->button_page_next_page_2, SIGNAL(released()), this, SLOT(onNextPage()));
	connect(ui->button_page_last_page_2, SIGNAL(released()), this, SLOT(onLastPage()));
	connect(ui->button_page_last_page_3, SIGNAL(released()), this, SLOT(onLastPage()));

	depth_clustering_first_return_ = std::make_shared<DepthClustering>();
	depth_clustering_second_return_ = std::make_shared<DepthClustering>();
	depth_clustering_custom_return_ = std::make_shared<DepthClustering>();
	depth_clustering_ = depth_clustering_first_return_;
	layout_ = VisualizationLayout();

	const auto &arguments = QCoreApplication::arguments();

	if (arguments.size() > 3)
	{
		layout_.configure(arguments.at(3).toStdString());
	}

	resetUI();

	if (arguments.size() > 2)
	{
		openDataset(arguments.at(1).toStdString(), arguments.at(2).toStdString());
	}
	else if (arguments.size() > 1)
	{
		openDataset(arguments.at(1).toStdString());
	}
}

void
Visualization::OnNewObjectReceived(const cv::Mat& image_segmentation, int id)
{
	if (ui->combo_layer_image_left->currentIndex()
			== static_cast<int>(VisualizationLayout::ImageViewerLayer::Segmentation)
			|| ui->combo_layer_image_middle->currentIndex()
					== static_cast<int>(VisualizationLayout::ImageViewerLayer::Segmentation)
			|| ui->combo_layer_image_right->currentIndex()
					== static_cast<int>(VisualizationLayout::ImageViewerLayer::Segmentation))
	{
		QImage qimage_segmentation = MatToQImage(
				AbstractImageLabeler::LabelsToColor(image_segmentation));

		scene_segmentation_.reset(new QGraphicsScene);
		scene_segmentation_->addPixmap(QPixmap::fromImage(qimage_segmentation));

		updateViewerImage();
	}
}

void
Visualization::OnNewObjectReceived(const std::pair<cv::Mat, cv::Mat>& images, int id)
{
	if (!initialized_
			&& ui->combo_lidar_return->currentIndex()
					== static_cast<int>(VisualizationLayout::LidarReturn::First)
			&& id == depth_clustering_second_return_->getDepthGroundRemover()->id())
	{
		return;
	}

	image_range_ground_ = images.first;
	image_range_no_ground_ = images.second;
}

Visualization::~Visualization()
{
}

bool
Visualization::eventFilter(QObject* object, QEvent* event)
{
	if (event->type() == QEvent::KeyPress)
	{
		QKeyEvent *keyEvent = static_cast<QKeyEvent*>(event);

		if (keyEvent->key() == Qt::Key_Right || keyEvent->key() == Qt::Key_Left)
		{
			keyPressEvent(keyEvent);
			return true;
		}
		else
		{
			return false;
		}
	}

	return false;
}

void
Visualization::keyPressEvent(QKeyEvent* event)
{
	if (!play_)
	{
		switch (event->key())
		{
		case Qt::Key_Right:
		{
			ui->spin_frame->setValue(ui->spin_frame->value() + 1);
			break;
		}
		case Qt::Key_Left:
		{
			ui->spin_frame->setValue(ui->spin_frame->value() - 1);
			break;
		}
		}
	}
}

void
Visualization::showEvent(QShowEvent* event)
{
	QWidget::showEvent(event);

	if (ui->combo_field_of_view->currentIndex()
			== static_cast<int>(VisualizationLayout::FieldOfView::Default))
	{
		ui->viewer_point_cloud->resetViewFOVDefault();
	}
	else if (ui->combo_field_of_view->currentIndex()
			== static_cast<int>(VisualizationLayout::FieldOfView::Camera))
	{
		ui->viewer_point_cloud->resetViewFOVCamera();
	}

	resize(minimumWidth(), minimumHeight());

	ui->viewer_image_camera->setFixedWidth(ui->viewer_image_camera->height() * 16 / 10);

	updateViewerImage();

	initialized_ = true;
}

void
Visualization::resizeEvent(QResizeEvent* event)
{
	QWidget::resizeEvent(event);

	ui->viewer_image_camera->setFixedWidth(ui->viewer_image_camera->height() * 16 / 10);

	ui->viewer_point_cloud->update();

	updateViewerImage();
}

void
Visualization::onOpen()
{
	const std::string dataset_path =
			QFileDialog::getExistingDirectory(this, "Open Dataset").toStdString();

	if (dataset_path != "")
	{
		openDataset(dataset_path, file_path_name_config_global_);
	}
}

void
Visualization::onPlay()
{
	if (play_)
	{
		return;
	}
	else
	{
		ui->button_play->setEnabled(false);
		ui->button_pause->setEnabled(true);
		ui->slider_frame->setEnabled(false);
		ui->spin_frame->setEnabled(false);
		play_ = true;
	}

	if (ui->slider_frame->value() >= ui->slider_frame->maximum())
	{
		ui->slider_frame->setValue(ui->slider_frame->minimum());
	}

	for (int i = ui->slider_frame->value(); i <= ui->slider_frame->maximum(); ++i)
	{
		if (!play_)
		{
			std::cout << std::endl << "[INFO]: Visualization stopped." << std::endl;
			return;
		}

		ui->slider_frame->setValue(i);
		ui->viewer_point_cloud->update();
		QApplication::processEvents();
	}

	onPause();

	std::cout << std::endl << "[INFO]: Visualization completed." << std::endl;
}

void
Visualization::onPause()
{
	if (play_)
	{
		play_ = false;
		ui->button_play->setEnabled(true);
		ui->button_pause->setEnabled(false);
		ui->slider_frame->setEnabled(true);
		ui->spin_frame->setEnabled(true);
	}
}

void
Visualization::onStop()
{
	onPause();

	ui->slider_frame->setValue(ui->slider_frame->minimum());
	ui->viewer_point_cloud->update();
	QApplication::processEvents();
}

void
Visualization::onSliderMovedTo(int frame_number)
{
	auto folder_reader_range = depth_clustering_->getFolderReaderRange();
	auto folder_reader_range_first_return = depth_clustering_first_return_->getFolderReaderRange();
	auto folder_reader_intensity = depth_clustering_->getFolderReaderIntensity();
	auto folder_reader_elongation = depth_clustering_->getFolderReaderElongation();
	auto folder_reader_camera = depth_clustering_->getFolderReaderCamera();
	const auto &frame_paths_names_range = folder_reader_range->GetAllFilePaths();
	const auto &frame_paths_names_range_first_return =
			folder_reader_range_first_return->GetAllFilePaths();
	const auto &frame_paths_names_intensity = folder_reader_intensity->GetAllFilePaths();
	const auto &frame_paths_names_elongation = folder_reader_elongation->GetAllFilePaths();
	const auto &frame_paths_names_camera = folder_reader_camera->GetAllFilePaths();
	std::string frame_path_name_range = "";
	std::string frame_path_name_range_first_return = "";
	std::string frame_path_name_intensity = "";
	std::string frame_path_name_elongation = "";
	std::string frame_path_name_camera = "";
	QList<int> splitter_viewer_sizes_collapsed;
	Timer timer;

	if (frame_paths_names_range.empty()
			|| frame_number >= static_cast<int>(frame_paths_names_range.size()))
	{
		std::cerr << "[WARN]: Range image missing in \"" << dataset_path_ << "\"." << std::endl;
		return;
	}
	else
	{
		frame_path_name_range = frame_paths_names_range[frame_number];
	}

	if (frame_paths_names_range_first_return.empty()
			|| frame_number >= static_cast<int>(frame_paths_names_range_first_return.size()))
	{
		std::cerr << "[WARN]: First return range image missing in \"" << dataset_path_ << "\"."
				<< std::endl;
		return;
	}
	else
	{
		frame_path_name_range_first_return = frame_paths_names_range_first_return[frame_number];
	}

	if (frame_paths_names_intensity.empty()
			|| frame_number >= static_cast<int>(frame_paths_names_range.size()))
	{
		std::cout << "[WARN]: Intensity image missing in \"" << dataset_path_ << "\"." << std::endl;
	}
	else
	{
		frame_path_name_intensity = frame_paths_names_intensity[frame_number];
	}

	if (frame_paths_names_elongation.empty()
			|| frame_number >= static_cast<int>(frame_paths_names_range.size()))
	{
		std::cout << "[WARN]: Elongation image missing in \"" << dataset_path_ << "\"."
				<< std::endl;
	}
	else
	{
		frame_path_name_elongation = frame_paths_names_elongation[frame_number];
	}

	splitter_viewer_sizes_collapsed.append(
			ui->splitter_viewer->sizes().at(0) + ui->splitter_viewer->sizes().at(1));
	splitter_viewer_sizes_collapsed.append(0);

	if (frame_paths_names_camera.empty()
			|| frame_number >= static_cast<int>(frame_paths_names_camera.size()))
	{
		std::cerr << "[WARN]: Camera image missing in \"" << dataset_path_ << "\"." << std::endl;

		if (!initialized_)
		{
			ui->splitter_viewer->setSizes(splitter_viewer_sizes_collapsed);
		}
	}
	else if (!initialized_ && !layout_.show_camera_frame)
	{
		ui->splitter_viewer->setSizes(splitter_viewer_sizes_collapsed);
	}
	else
	{
		frame_path_name_camera = frame_paths_names_camera[frame_number];
	}

	std::cout << std::endl;
	depth_clustering_->processOneFrameForDataset(frame_path_name_range, frame_path_name_intensity,
			frame_path_name_elongation);

	if (!initialized_
			|| (ui->combo_layer_point_cloud->currentIndex()
					== static_cast<int>(VisualizationLayout::PointCloudViewerLayer::Second_Return)
					&& ui->combo_lidar_return->currentIndex()
							== static_cast<int>(VisualizationLayout::LidarReturn::First)))
	{
		auto folder_reader_range_second_return =
				depth_clustering_second_return_->getFolderReaderRange();
		const auto &frame_paths_names_range_second_return =
				folder_reader_range_second_return->GetAllFilePaths();

		if (frame_paths_names_range_second_return.empty()
				|| frame_number >= static_cast<int>(frame_paths_names_range_second_return.size()))
		{
			std::cerr << "[WARN]: Second return range image missing in \"" << dataset_path_ << "\"."
					<< std::endl;
		}
		else
		{
			depth_clustering_second_return_->processOneFrameForDataset(
					frame_paths_names_range_second_return[frame_number]);
		}
	}

	ground_truth_frame_cube_ = depth_clustering_->getGroundTruthFrameCube(
			frame_paths_names_range_first_return[frame_number]);

	ground_truth_frame_flat_ = depth_clustering_->getGroundTruthFrameFlat(
			frame_paths_names_range_first_return[frame_number]);

	timer.start();

	updateViewerPointCloud();

	std::cout << "[INFO]: Point cloud viewer updated: " << timer.measure() << " us." << std::endl;

	updateViewerImageScene(frame_path_name_camera);
	updateViewerImage();

	std::cout << "[INFO]: Image viewers updated: " << timer.measure() << " us." << std::endl;
}

void
Visualization::onParameterUpdated()
{
	switch (static_cast<VisualizationLayout::LidarReturn>(ui->combo_lidar_return->currentIndex()))
	{
	case VisualizationLayout::LidarReturn::First:
	{
		depth_clustering_ = depth_clustering_first_return_;
		break;
	}
	case VisualizationLayout::LidarReturn::Second:
	{
		depth_clustering_ = depth_clustering_second_return_;
		break;
	}
	case VisualizationLayout::LidarReturn::Custom:
	{
		depth_clustering_ = depth_clustering_custom_return_;
		break;
	}
	default:
	{
		depth_clustering_ = depth_clustering_first_return_;
		break;
	}
	}

	DepthClusteringParameter parameter = depth_clustering_->getParameter();

	try
	{
		parameter.score_type_point =
				static_cast<Score::TypePoint>(ui->combo_point_score_type->currentIndex());
	} catch (const std::exception &e)
	{
		std::cout << "[WARN]: Unknown point score type." << std::endl;
		parameter.score_type_point = Score::TypePoint::Type_1;
	}

	try
	{
		parameter.score_type_cluster =
				static_cast<Score::TypeCluster>(ui->combo_cluster_score_type->currentIndex());
	} catch (const std::exception &e)
	{
		std::cout << "[WARN]: Unknown cluster score type." << std::endl;
		parameter.score_type_cluster = Score::TypeCluster::Type_1;
	}

	try
	{
		parameter.score_type_frame =
				static_cast<Score::TypeFrame>(ui->combo_frame_score_type->currentIndex());
	} catch (const std::exception &e)
	{
		std::cout << "[WARN]: Unknown frame score type." << std::endl;
		parameter.score_type_frame = Score::TypeFrame::Type_1;
	}

	parameter.use_score_filter = static_cast<bool>(ui->combo_score_filter->currentIndex());
	parameter.score_filter_threshold = ui->spin_score_filter_threshold->value();
	parameter.angle_ground_removal = Radians::FromDegrees(ui->spin_angle_ground_removal->value());
	parameter.size_smooth_window = ui->combo_size_smooth_window->currentIndex() * 2 + 5;
	parameter.score_clustering = ui->spin_clustering_score_threshold->value();

	switch (ui->combo_difference_type->currentIndex())
	{
	case 0:
	{
		parameter.angle_clustering = Radians::FromDegrees(
				ui->spin_clustering_depth_threshold->value());
		break;
	}
	case 1:
	{
		parameter.angle_clustering = Radians::FromDegrees(
				ui->spin_clustering_depth_threshold->value());
		break;
	}
	case 2:
	{
		parameter.distance_clustering = ui->spin_clustering_depth_threshold->value();
		break;
	}
	case 3:
	{
		parameter.distance_clustering = ui->spin_clustering_depth_threshold->value();
		break;
	}
	case 4:
	{
		parameter.distance_clustering = ui->spin_clustering_depth_threshold->value();
		break;
	}
	default:
	{
		parameter.angle_clustering = Radians::FromDegrees(
				ui->spin_clustering_depth_threshold->value());
		break;
	}
	}

	parameter.size_cluster_min = ui->spin_size_cluster_min->value();
	parameter.size_cluster_max = ui->spin_size_cluster_max->value();

	BoundingBox::Type bounding_box_type = BoundingBox::Type::Cube;

	switch (static_cast<VisualizationLayout::BoundingBoxType>(ui->combo_bounding_box_type->currentIndex()))
	{
	case VisualizationLayout::BoundingBoxType::Cube:
	{
		bounding_box_type = BoundingBox::Type::Cube;
		break;
	}
	case VisualizationLayout::BoundingBoxType::Polygon:
	{
		bounding_box_type = BoundingBox::Type::Polygon;
		break;
	}
	case VisualizationLayout::BoundingBoxType::None:
	{
		bounding_box_type = parameter.bounding_box_type;
		break;
	}
	default:
	{
		bounding_box_type = BoundingBox::Type::Polygon;
		break;
	}
	}

	parameter.bounding_box_type = bounding_box_type;

	if (!initialized_)
	{
		parameter.use_camera_fov = static_cast<bool>(ui->combo_field_of_view->currentIndex());

		switch (static_cast<VisualizationLayout::FieldOfView>(ui->combo_field_of_view->currentIndex()))
		{
		case VisualizationLayout::FieldOfView::Default:
		{
			ui->viewer_point_cloud->resetViewFOVDefault();
			break;
		}
		case VisualizationLayout::FieldOfView::Camera:
		{
			ui->viewer_point_cloud->resetViewFOVCamera();
			break;
		}
		default:
		{
			ui->viewer_point_cloud->resetViewFOVDefault();
			break;
		}
		}
	}

	depth_clustering_first_return_->setParameter(parameter);
	depth_clustering_first_return_->getDepthGroundRemover()->DepthGroundRemover::SenderTImage::AddClient( // @suppress("Method cannot be resolved")
			this);
	depth_clustering_first_return_->getClusterer()->SetLabelImageClient(this);

	depth_clustering_second_return_->setParameter(parameter);
	depth_clustering_second_return_->getDepthGroundRemover()->DepthGroundRemover::SenderTImage::AddClient( // @suppress("Method cannot be resolved")
			this);
	depth_clustering_second_return_->getClusterer()->SetLabelImageClient(this);

	depth_clustering_custom_return_->setParameter(parameter);
	depth_clustering_custom_return_->getDepthGroundRemover()->DepthGroundRemover::SenderTImage::AddClient( // @suppress("Method cannot be resolved")
			this);
	depth_clustering_custom_return_->getClusterer()->SetLabelImageClient(this);

	resetViewer();
	onSliderMovedTo(ui->slider_frame->value());

	std::cout << "[INFO]: Updated parameters." << std::endl;
}

void
Visualization::onLoadGlobalConfiguration()
{
	const std::string file_path_name_config_global = QFileDialog::getOpenFileName(this,
			"Load Global Configuration File", QString(), tr("JSON Files (*.json)")).toStdString();

	if (file_path_name_config_global != "")
	{
		openDataset(dataset_path_, file_path_name_config_global);
	}
}

void
Visualization::onLoadLayoutConfiguration()
{
	const std::string file_path_name_config_layout = QFileDialog::getOpenFileName(this,
			"Load Layout Configuration File", QString(), tr("JSON Files (*.json)")).toStdString();

	if (file_path_name_config_layout != "")
	{
		layout_.configure(file_path_name_config_layout);
		initializeUI();
	}
}

void
Visualization::onSplitterViewerMoved()
{
	static bool viewer_image_camera_was_collapsed;

	if (ui->splitter_viewer->sizes().at(1) != 0)
	{
		if (viewer_image_camera_was_collapsed
				&& !ui->viewer_image_camera->visibleRegion().isEmpty())
		{
			onSliderMovedTo(ui->slider_frame->value());
		}

		viewer_image_camera_was_collapsed = false;
	}
	else
	{
		viewer_image_camera_was_collapsed = true;
	}
}

void
Visualization::onLayerPointCloudUpdated()
{
	Timer timer;

	timer.start();

	updateViewerPointCloud();

	std::cout << "[INFO]: Point cloud viewer updated: " << timer.measure() << " us." << std::endl;
}

void
Visualization::onLayerImageUpdated()
{
	Timer timer;

	timer.start();

	updateViewerImageScene();
	updateViewerImage();

	std::cout << "[INFO]: Image viewers updated: " << timer.measure() << " us." << std::endl;
}

void
Visualization::onDifferenceTypeUpdated()
{
	DepthClusteringParameter parameter = depth_clustering_->getParameter();

	DiffFactory::DiffType difference_type = DiffFactory::DiffType::ANGLES_PRECOMPUTED;

	switch (ui->combo_difference_type->currentIndex())
	{
	case 0:
	{
		difference_type = DiffFactory::DiffType::ANGLES;
		ui->spin_clustering_depth_threshold->setValue(parameter.angle_clustering.ToDegrees());
		break;
	}
	case 1:
	{
		difference_type = DiffFactory::DiffType::ANGLES_PRECOMPUTED;
		ui->spin_clustering_depth_threshold->setValue(parameter.angle_clustering.ToDegrees());
		break;
	}
	case 2:
	{
		difference_type = DiffFactory::DiffType::LINE_DIST;
		ui->spin_clustering_depth_threshold->setValue(parameter.distance_clustering);
		break;
	}
	case 3:
	{
		difference_type = DiffFactory::DiffType::LINE_DIST_PRECOMPUTED;
		ui->spin_clustering_depth_threshold->setValue(parameter.distance_clustering);
		break;
	}
	case 4:
	{
		difference_type = DiffFactory::DiffType::SIMPLE;
		ui->spin_clustering_depth_threshold->setValue(parameter.distance_clustering);
		break;
	}
	default:
	{
		difference_type = DiffFactory::DiffType::ANGLES_PRECOMPUTED;
		ui->spin_clustering_depth_threshold->setValue(parameter.angle_clustering.ToDegrees());
		break;
	}
	}

	parameter.difference_type = difference_type;

	depth_clustering_first_return_->setParameter(parameter);
	depth_clustering_first_return_->getDepthGroundRemover()->DepthGroundRemover::SenderTImage::AddClient( // @suppress("Method cannot be resolved")
			this);
	depth_clustering_first_return_->getClusterer()->SetLabelImageClient(this);

	depth_clustering_second_return_->setParameter(parameter);
	depth_clustering_second_return_->getDepthGroundRemover()->DepthGroundRemover::SenderTImage::AddClient( // @suppress("Method cannot be resolved")
			this);
	depth_clustering_second_return_->getClusterer()->SetLabelImageClient(this);

	depth_clustering_custom_return_->setParameter(parameter);
	depth_clustering_custom_return_->getDepthGroundRemover()->DepthGroundRemover::SenderTImage::AddClient( // @suppress("Method cannot be resolved")
			this);
	depth_clustering_custom_return_->getClusterer()->SetLabelImageClient(this);

	resetViewer();
	onSliderMovedTo(ui->slider_frame->value());

	std::cout << "[INFO]: Updated difference type." << std::endl;
}

void
Visualization::onFieldOfViewUpdated()
{
	DepthClusteringParameter parameter = depth_clustering_->getParameter();

	parameter.use_camera_fov = static_cast<bool>(ui->combo_field_of_view->currentIndex());

	switch (static_cast<VisualizationLayout::FieldOfView>(ui->combo_field_of_view->currentIndex()))
	{
	case VisualizationLayout::FieldOfView::Default:
	{
		ui->viewer_point_cloud->resetViewFOVDefault();
		break;
	}
	case VisualizationLayout::FieldOfView::Camera:
	{
		ui->viewer_point_cloud->resetViewFOVCamera();
		break;
	}
	default:
	{
		ui->viewer_point_cloud->resetViewFOVDefault();
		break;
	}
	}

	depth_clustering_first_return_->setParameter(parameter);
	depth_clustering_first_return_->getDepthGroundRemover()->DepthGroundRemover::SenderTImage::AddClient( // @suppress("Method cannot be resolved")
			this);
	depth_clustering_first_return_->getClusterer()->SetLabelImageClient(this);

	depth_clustering_second_return_->setParameter(parameter);
	depth_clustering_second_return_->getDepthGroundRemover()->DepthGroundRemover::SenderTImage::AddClient( // @suppress("Method cannot be resolved")
			this);
	depth_clustering_second_return_->getClusterer()->SetLabelImageClient(this);

	depth_clustering_custom_return_->setParameter(parameter);
	depth_clustering_custom_return_->getDepthGroundRemover()->DepthGroundRemover::SenderTImage::AddClient( // @suppress("Method cannot be resolved")
			this);
	depth_clustering_custom_return_->getClusterer()->SetLabelImageClient(this);

	resetViewer();
	onSliderMovedTo(ui->slider_frame->value());

	std::cout << "[INFO]: Updated field of view." << std::endl;
}

void
Visualization::onNextPage()
{
	if (ui->stacked_settings->currentIndex() >= ui->stacked_settings->count() - 1)
	{
		return;
	}

	ui->stacked_settings->setCurrentIndex(ui->stacked_settings->currentIndex() + 1);
}

void
Visualization::onLastPage()
{
	if (ui->stacked_settings->currentIndex() <= 0)
	{
		return;
	}

	ui->stacked_settings->setCurrentIndex(ui->stacked_settings->currentIndex() - 1);
}

void
Visualization::openDataset(const std::string& dataset_path,
		const std::string& file_path_name_config_global)
{
	if (dataset_path == "")
	{
		std::cerr << "[ERROR]: Empty dataset path." << std::endl;
		return;
	}

	resetUI();

	dataset_path_ = dataset_path;
	file_path_name_config_global_ = file_path_name_config_global;
	play_ = false;

	if (!depth_clustering_first_return_ || !depth_clustering_second_return_
			|| !depth_clustering_custom_return_)
	{
		std::cerr << "[ERROR]: API missing." << std::endl;
		return;
	}

	auto depth_clustering_first_return_initialized =
			depth_clustering_first_return_->initializeForDataset(dataset_path_,
					file_path_name_config_global_, "first_return");
	auto depth_clustering_second_return_initialized =
			depth_clustering_second_return_->initializeForDataset(dataset_path_,
					file_path_name_config_global_, "second_return");
	auto depth_clustering_custom_return_initialized =
			depth_clustering_custom_return_->initializeForDataset(dataset_path_,
					file_path_name_config_global_, "custom_return");

	if (!depth_clustering_first_return_initialized || !depth_clustering_second_return_initialized
			|| !depth_clustering_custom_return_initialized)
	{
		std::cerr << "[ERROR]: Failed to open dataset at \"" << dataset_path_ << "\"." << std::endl;
		return;
	}

	std::cout << "[INFO]: Opened dataset at \"" << dataset_path_ << "\"." << std::endl;

	initializeUI();
}

void
Visualization::updateViewerPointCloud()
{
	ui->viewer_point_cloud->Clear();

	switch (static_cast<VisualizationLayout::PointCloudViewerLayer>(ui->combo_layer_point_cloud->currentIndex()))
	{
	case VisualizationLayout::PointCloudViewerLayer::Ground_Removal:
	{
		const ProjectionParams parameter_projection_lidar =
				*depth_clustering_->getLidarProjectionParameter();
		const auto cloud_ground = Cloud::FromImage(image_range_ground_, parameter_projection_lidar);
		const auto cloud_no_ground = Cloud::FromImage(image_range_no_ground_,
				parameter_projection_lidar);

		if (!cloud_ground || !cloud_no_ground)
		{
			std::cerr << "[WARN]: Ground cloud missing." << std::endl;

			const auto cloud = depth_clustering_->getCloud();

			if (!cloud)
			{
				std::cerr << "[ERROR]: Cloud missing." << std::endl;
			}
			else
			{
				ui->viewer_point_cloud->AddDrawable(DrawableCloud::FromCloud(cloud));
			}
		}
		else
		{
			ui->viewer_point_cloud->AddDrawable(
					DrawableCloud::FromCloud(cloud_ground, Eigen::Vector3f(1, 0, 0)));
			ui->viewer_point_cloud->AddDrawable(DrawableCloud::FromCloud(cloud_no_ground));
		}

		break;
	}
	case VisualizationLayout::PointCloudViewerLayer::Second_Return:
	{
		const auto cloud_second_return = depth_clustering_second_return_->getCloud();

		if (!cloud_second_return)
		{
			std::cerr << "[WARN]: Second return cloud missing." << std::endl;
		}
		else
		{
			ui->viewer_point_cloud->AddDrawable(
					DrawableCloud::FromCloud(cloud_second_return, Eigen::Vector3f(0, 1, 0)));
		}

		if (ui->combo_lidar_return->currentIndex()
				== static_cast<int>(VisualizationLayout::LidarReturn::First))
		{
			const auto cloud = depth_clustering_->getCloud();

			if (!cloud)
			{
				std::cerr << "[ERROR]: Cloud missing." << std::endl;
			}
			else
			{
				ui->viewer_point_cloud->AddDrawable(DrawableCloud::FromCloud(cloud));
			}
		}

		break;
	}
	case VisualizationLayout::PointCloudViewerLayer::Intensity:
	{
		const auto cloud = depth_clustering_->getCloud();

		if (!cloud)
		{
			std::cerr << "[ERROR]: Cloud missing." << std::endl;
		}
		else
		{
			ui->viewer_point_cloud->AddDrawable(DrawableCloud::FromCloudWithIntensity(cloud));
		}

		break;
	}
	case VisualizationLayout::PointCloudViewerLayer::Elongation:
	{
		const auto cloud = depth_clustering_->getCloud();

		if (!cloud)
		{
			std::cerr << "[ERROR]: Cloud missing." << std::endl;
		}
		else
		{
			ui->viewer_point_cloud->AddDrawable(DrawableCloud::FromCloudWithElongation(cloud));
		}

		break;
	}
	case VisualizationLayout::PointCloudViewerLayer::Point_Score:
	{
		const auto cloud = depth_clustering_->getCloud();

		if (!cloud)
		{
			std::cerr << "[ERROR]: Cloud missing." << std::endl;
		}
		else
		{
			ui->viewer_point_cloud->AddDrawable(DrawableCloud::FromCloudWithScore(cloud));
		}

		break;
	}
	case VisualizationLayout::PointCloudViewerLayer::Cluster_Score:
	{
		const auto frame_cluster = depth_clustering_->getBoundingBox()->getFrameCluster();

		if (frame_cluster)
		{
			for (const auto &cluster : *frame_cluster)
			{
				const auto cloud = Cloud::Ptr(new Cloud(std::get<0>(cluster)));
				const auto &score = std::get<1>(cluster);
				ui->viewer_point_cloud->AddDrawable(
						DrawableCloud::FromCloudWithValue(cloud, score));
			}
		}
		else
		{
			std::cout << "[WARN]: Cluster frame missing." << std::endl;
		}

		const auto cloud = depth_clustering_->getCloud();

		if (!cloud)
		{
			std::cerr << "[ERROR]: Cloud missing." << std::endl;
		}
		else
		{
			ui->viewer_point_cloud->AddDrawable(DrawableCloud::FromCloud(cloud));
		}

		break;
	}
	case VisualizationLayout::PointCloudViewerLayer::Frame_Score:
	{
		const auto score = depth_clustering_->getBoundingBox()->getFrameScore();

		const auto cloud = depth_clustering_->getCloud();

		if (!cloud)
		{
			std::cerr << "[ERROR]: Cloud missing." << std::endl;
		}
		else
		{
			ui->viewer_point_cloud->AddDrawable(DrawableCloud::FromCloudWithValue(cloud, score));
		}

		break;
	}
	default:
	{
		const auto cloud = depth_clustering_->getCloud();

		if (!cloud)
		{
			std::cerr << "[ERROR]: Cloud missing." << std::endl;
		}
		else
		{
			ui->viewer_point_cloud->AddDrawable(DrawableCloud::FromCloud(cloud));
		}

		break;
	}
	}

	if (static_cast<bool>(ui->combo_bounding_box_type->currentIndex()))
	{
		if (ground_truth_frame_cube_)
		{
			for (const auto &ground_truth_cube : *ground_truth_frame_cube_)
			{
				auto center = std::get<0>(ground_truth_cube);
				auto extent = std::get<1>(ground_truth_cube);
				auto rotation = std::get<2>(ground_truth_cube);

				auto cube_drawable = DrawableCube::Create(center, extent, Eigen::Vector3f(0, 1, 0),
						rotation);

				ui->viewer_point_cloud->AddDrawable(std::move(cube_drawable));
			}
		}
		else
		{
			std::cout << "[WARN]: Cube ground truth frame missing." << std::endl;
		}

		auto bounding_box = depth_clustering_->getBoundingBox();
		const auto &parameter = depth_clustering_->getParameter();

		switch (parameter.bounding_box_type)
		{
		case BoundingBox::Type::Cube:
		{
			auto frame_cube = bounding_box->getFrameCube();

			if (!frame_cube)
			{
				std::cerr << "[ERROR]: Cube frame missing." << std::endl;
				break;
			}

			for (const auto &cube : *frame_cube)
			{
				auto center = std::get<0>(cube);
				auto extent = std::get<1>(cube);

				auto cube_drawable = DrawableCube::Create(center, extent, Eigen::Vector3f(1, 0, 0));

				ui->viewer_point_cloud->AddDrawable(std::move(cube_drawable));
			}

			break;
		}
		case BoundingBox::Type::Polygon:
		{
			auto frame_polygon = bounding_box->getFramePolygon();

			if (!frame_polygon)
			{
				std::cerr << "[ERROR]: Polygon frame missing." << std::endl;
				break;
			}

			for (const auto &polygon : *frame_polygon)
			{
				auto hull = std::get<0>(polygon);
				auto diff_z = std::get<1>(polygon);

				auto polygon_drawable = DrawablePolygon3d::Create(hull, diff_z,
						Eigen::Vector3f(1, 0, 0));

				ui->viewer_point_cloud->AddDrawable(std::move(polygon_drawable));
			}

			break;
		}
		case BoundingBox::Type::Flat:
		{
			std::cerr << "[ERROR]: Cannot display flat bounding box in lidar visualizer."
					<< std::endl;
			break;
		}
		default:
		{
			break;
		}
		}
	}

	ui->viewer_point_cloud->update();
}

void
Visualization::updateViewerImageScene(const std::string& frame_path_name_camera)
{
	if (frame_path_name_camera != "")
	{
		if (!initialized_ || !ui->viewer_image_camera->visibleRegion().isEmpty())
		{
			auto image_camera = depth_clustering_->getImageCamera(frame_path_name_camera);
			auto bounding_box_frame_flat = depth_clustering_->getBoundingBoxFrameFlat();

			QImage qimage_camera = MatToQImage(image_camera);

			if (static_cast<bool>(ui->combo_bounding_box_type->currentIndex()))
			{
				QFont font;
				QPen pen_bounding_box;
				QPen pen_ground_truth;
				QPainter painter(&qimage_camera);

				font.setPixelSize(30);
				font.setBold(QFont::Bold);

				pen_bounding_box.setWidth(5);
				pen_bounding_box.setColor(Qt::red);

				pen_ground_truth.setWidth(5);
				pen_ground_truth.setColor(Qt::green);

				painter.setFont(font);
				painter.setRenderHints(
						QPainter::SmoothPixmapTransform | QPainter::Antialiasing
								| QPainter::HighQualityAntialiasing);

				if (ground_truth_frame_flat_)
				{
					for (const auto &ground_truth_flat : *ground_truth_frame_flat_)
					{
						painter.setPen(pen_ground_truth);

						painter.drawRect(std::get<0>(ground_truth_flat).x(),
								std::get<0>(ground_truth_flat).y(),
								std::get<1>(ground_truth_flat).x()
										- std::get<0>(ground_truth_flat).x(),
								std::get<1>(ground_truth_flat).y()
										- std::get<0>(ground_truth_flat).y());

						painter.drawText(std::get<0>(ground_truth_flat).x(),
								std::get<1>(ground_truth_flat).y() + 30,
								QString::fromStdString(
										"depth: "
												+ std::to_string(std::get<2>(ground_truth_flat))));
					}
				}
				else
				{
					std::cout << "[WARN]: Flat ground truth frame missing." << std::endl;
				}

				if (bounding_box_frame_flat)
				{
					for (const auto &bounding_box_flat : *bounding_box_frame_flat)
					{
						painter.setPen(pen_bounding_box);

						painter.drawRect(std::get<0>(bounding_box_flat).x(),
								std::get<0>(bounding_box_flat).y(),
								std::get<1>(bounding_box_flat).x()
										- std::get<0>(bounding_box_flat).x(),
								std::get<1>(bounding_box_flat).y()
										- std::get<0>(bounding_box_flat).y());

						painter.drawText(std::get<0>(bounding_box_flat).x(),
								std::get<0>(bounding_box_flat).y() - 15,
								QString::fromStdString(
										"depth: "
												+ std::to_string(std::get<2>(bounding_box_flat))));
					}
				}
				else
				{
					std::cout << "[WARN]: Flat bounding box frame missing." << std::endl;
				}

				painter.end();
			}

			scene_camera_.reset(new QGraphicsScene);
			scene_camera_->addPixmap(QPixmap::fromImage(qimage_camera));
		}
	}

	if (ui->combo_layer_image_left->currentIndex()
			== static_cast<int>(VisualizationLayout::ImageViewerLayer::Clustering)
			|| ui->combo_layer_image_middle->currentIndex()
					== static_cast<int>(VisualizationLayout::ImageViewerLayer::Clustering)
			|| ui->combo_layer_image_right->currentIndex()
					== static_cast<int>(VisualizationLayout::ImageViewerLayer::Clustering))
	{
		auto image_range = depth_clustering_->getImageRange();
		auto difference_type = depth_clustering_->getParameter().difference_type;
		auto parameter_projection_lidar = depth_clustering_->getLidarProjectionParameter();
		auto difference_helper = DiffFactory::Build(difference_type, &image_range,
				parameter_projection_lidar.get());
		QImage qimage_difference = MatToQImage(difference_helper->Visualize());

		scene_difference_.reset(new QGraphicsScene);
		scene_difference_->addPixmap(QPixmap::fromImage(qimage_difference));
	}

	if (ui->combo_layer_image_left->currentIndex()
			== static_cast<int>(VisualizationLayout::ImageViewerLayer::Range)
			|| ui->combo_layer_image_middle->currentIndex()
					== static_cast<int>(VisualizationLayout::ImageViewerLayer::Range)
			|| ui->combo_layer_image_right->currentIndex()
					== static_cast<int>(VisualizationLayout::ImageViewerLayer::Range))
	{
		auto parameter = depth_clustering_->getParameter();
		auto image_range = depth_clustering_->getImageRange();
		QImage qimage_range;

		if (parameter.dataset_file_type == ".png")
		{
			qimage_range = MatPNGRangeToQImage(image_range);
		}
		else if (parameter.dataset_file_type == ".tiff")
		{
			qimage_range = MatTIFFRangeToQImage(image_range);
		}
		else
		{
			std::cout << "[WARN]: Unknown dataset file type." << std::endl;
		}

		scene_range_.reset(new QGraphicsScene);
		scene_range_->addPixmap(QPixmap::fromImage(qimage_range));
	}

	if (ui->combo_layer_image_left->currentIndex()
			== static_cast<int>(VisualizationLayout::ImageViewerLayer::Intensity)
			|| ui->combo_layer_image_middle->currentIndex()
					== static_cast<int>(VisualizationLayout::ImageViewerLayer::Intensity)
			|| ui->combo_layer_image_right->currentIndex()
					== static_cast<int>(VisualizationLayout::ImageViewerLayer::Intensity))
	{
		auto parameter = depth_clustering_->getParameter();
		auto image_intensity = depth_clustering_->getImageIntensity();
		const auto intensity_norm_factor =
				depth_clustering_->getLidarProjectionParameter()->getProjectionParamsRaw()->intensity_norm_factor;
		QImage qimage_intensity;

		if (parameter.dataset_file_type == ".png")
		{
			std::cerr
					<< "[ERROR]: The processing of \".png\" type intensity images is not implemented."
					<< std::endl;
		}
		else if (parameter.dataset_file_type == ".tiff")
		{
			qimage_intensity = MatTIFFIntensityToQImage(image_intensity, intensity_norm_factor);
		}
		else
		{
			std::cout << "[WARN]: Unknown dataset file type." << std::endl;
		}

		scene_intensity_.reset(new QGraphicsScene);
		scene_intensity_->addPixmap(QPixmap::fromImage(qimage_intensity));
	}

	if (ui->combo_layer_image_left->currentIndex()
			== static_cast<int>(VisualizationLayout::ImageViewerLayer::Elongation)
			|| ui->combo_layer_image_middle->currentIndex()
					== static_cast<int>(VisualizationLayout::ImageViewerLayer::Elongation)
			|| ui->combo_layer_image_right->currentIndex()
					== static_cast<int>(VisualizationLayout::ImageViewerLayer::Elongation))
	{
		auto parameter = depth_clustering_->getParameter();
		auto image_elongation = depth_clustering_->getImageElongation();
		const auto elongation_norm_factor =
				depth_clustering_->getLidarProjectionParameter()->getProjectionParamsRaw()->elongation_norm_factor;
		QImage qimage_elongation;

		if (parameter.dataset_file_type == ".png")
		{
			std::cerr
					<< "[ERROR]: The processing of \".png\" type elongation images is not implemented."
					<< std::endl;
		}
		else if (parameter.dataset_file_type == ".tiff")
		{
			qimage_elongation = MatTIFFElongationToQImage(image_elongation, elongation_norm_factor);
		}
		else
		{
			std::cout << "[WARN]: Unknown dataset file type." << std::endl;
		}

		scene_elongation_.reset(new QGraphicsScene);
		scene_elongation_->addPixmap(QPixmap::fromImage(qimage_elongation));
	}
}

void
Visualization::updateViewerImage()
{
	if (!initialized_ || !ui->viewer_image_camera->visibleRegion().isEmpty())
	{
		ui->viewer_image_camera->setScene(scene_camera_.get());
		ui->viewer_image_camera->fitInView(scene_camera_->itemsBoundingRect());
	}

	switch (static_cast<VisualizationLayout::ImageViewerLayer>(ui->combo_layer_image_left->currentIndex()))
	{
	case VisualizationLayout::ImageViewerLayer::Clustering:
	{
		ui->viewer_image_left->setScene(scene_difference_.get());
		ui->viewer_image_left->fitInView(scene_difference_->itemsBoundingRect());
		break;
	}
	case VisualizationLayout::ImageViewerLayer::Segmentation:
	{
		ui->viewer_image_left->setScene(scene_segmentation_.get());
		ui->viewer_image_left->fitInView(scene_segmentation_->itemsBoundingRect());
		break;
	}
	case VisualizationLayout::ImageViewerLayer::Range:
	{
		ui->viewer_image_left->setScene(scene_range_.get());
		ui->viewer_image_left->fitInView(scene_range_->itemsBoundingRect());
		break;
	}
	case VisualizationLayout::ImageViewerLayer::Intensity:
	{
		ui->viewer_image_left->setScene(scene_intensity_.get());
		ui->viewer_image_left->fitInView(scene_intensity_->itemsBoundingRect());
		break;
	}
	case VisualizationLayout::ImageViewerLayer::Elongation:
	{
		ui->viewer_image_left->setScene(scene_elongation_.get());
		ui->viewer_image_left->fitInView(scene_elongation_->itemsBoundingRect());
		break;
	}
	default:
	{
		ui->viewer_image_left->setScene(scene_empty_.get());
		ui->viewer_image_left->fitInView(scene_empty_->itemsBoundingRect());
		break;
	}
	}

	switch (static_cast<VisualizationLayout::ImageViewerLayer>(ui->combo_layer_image_middle->currentIndex()))
	{
	case VisualizationLayout::ImageViewerLayer::Clustering:
	{
		ui->viewer_image_middle->setScene(scene_difference_.get());
		ui->viewer_image_middle->fitInView(scene_difference_->itemsBoundingRect());
		break;
	}
	case VisualizationLayout::ImageViewerLayer::Segmentation:
	{
		ui->viewer_image_middle->setScene(scene_segmentation_.get());
		ui->viewer_image_middle->fitInView(scene_segmentation_->itemsBoundingRect());
		break;
	}
	case VisualizationLayout::ImageViewerLayer::Range:
	{
		ui->viewer_image_middle->setScene(scene_range_.get());
		ui->viewer_image_middle->fitInView(scene_range_->itemsBoundingRect());
		break;
	}
	case VisualizationLayout::ImageViewerLayer::Intensity:
	{
		ui->viewer_image_middle->setScene(scene_intensity_.get());
		ui->viewer_image_middle->fitInView(scene_intensity_->itemsBoundingRect());
		break;
	}
	case VisualizationLayout::ImageViewerLayer::Elongation:
	{
		ui->viewer_image_middle->setScene(scene_elongation_.get());
		ui->viewer_image_middle->fitInView(scene_elongation_->itemsBoundingRect());
		break;
	}
	default:
	{
		ui->viewer_image_middle->setScene(scene_empty_.get());
		ui->viewer_image_middle->fitInView(scene_empty_->itemsBoundingRect());
		break;
	}
	}

	switch (static_cast<VisualizationLayout::ImageViewerLayer>(ui->combo_layer_image_right->currentIndex()))
	{
	case VisualizationLayout::ImageViewerLayer::Clustering:
	{
		ui->viewer_image_right->setScene(scene_difference_.get());
		ui->viewer_image_right->fitInView(scene_difference_->itemsBoundingRect());
		break;
	}
	case VisualizationLayout::ImageViewerLayer::Segmentation:
	{
		ui->viewer_image_right->setScene(scene_segmentation_.get());
		ui->viewer_image_right->fitInView(scene_segmentation_->itemsBoundingRect());
		break;
	}
	case VisualizationLayout::ImageViewerLayer::Range:
	{
		ui->viewer_image_right->setScene(scene_range_.get());
		ui->viewer_image_right->fitInView(scene_range_->itemsBoundingRect());
		break;
	}
	case VisualizationLayout::ImageViewerLayer::Intensity:
	{
		ui->viewer_image_right->setScene(scene_intensity_.get());
		ui->viewer_image_right->fitInView(scene_intensity_->itemsBoundingRect());
		break;
	}
	case VisualizationLayout::ImageViewerLayer::Elongation:
	{
		ui->viewer_image_right->setScene(scene_elongation_.get());
		ui->viewer_image_right->fitInView(scene_elongation_->itemsBoundingRect());
		break;
	}
	default:
	{
		ui->viewer_image_right->setScene(scene_empty_.get());
		ui->viewer_image_right->fitInView(scene_empty_->itemsBoundingRect());
		break;
	}
	}
}

void
Visualization::resetViewer()
{
	ui->viewer_point_cloud->Clear();
	ui->viewer_point_cloud->update();

	scene_empty_.reset(new QGraphicsScene);
	scene_empty_->addPixmap(QPixmap::fromImage(QImage()));
	scene_camera_.reset(new QGraphicsScene);
	scene_camera_->addPixmap(QPixmap::fromImage(QImage()));
	scene_difference_.reset(new QGraphicsScene);
	scene_difference_->addPixmap(QPixmap::fromImage(QImage()));
	scene_segmentation_.reset(new QGraphicsScene);
	scene_segmentation_->addPixmap(QPixmap::fromImage(QImage()));
	scene_range_.reset(new QGraphicsScene);
	scene_range_->addPixmap(QPixmap::fromImage(QImage()));
	scene_intensity_.reset(new QGraphicsScene);
	scene_intensity_->addPixmap(QPixmap::fromImage(QImage()));
	scene_elongation_.reset(new QGraphicsScene);
	scene_elongation_->addPixmap(QPixmap::fromImage(QImage()));

	updateViewerImage();
}

void
Visualization::resetUI()
{
	resetViewer();

	ui->button_play->setEnabled(false);
	ui->button_pause->setEnabled(false);
	ui->button_stop->setEnabled(false);
	ui->slider_frame->setEnabled(false);
	ui->spin_frame->setEnabled(false);
	ui->spin_angle_ground_removal->setEnabled(false);
	ui->combo_size_smooth_window->setEnabled(false);
	ui->spin_clustering_depth_threshold->setEnabled(false);
	ui->spin_clustering_score_threshold->setEnabled(false);
	ui->combo_difference_type->setEnabled(false);
	ui->spin_size_cluster_min->setEnabled(false);
	ui->spin_size_cluster_max->setEnabled(false);
	ui->button_global_configuration->setEnabled(false);
	ui->button_layout_configuration->setEnabled(false);
	ui->combo_layer_point_cloud->setEnabled(false);
	ui->combo_point_score_type->setEnabled(false);
	ui->combo_cluster_score_type->setEnabled(false);
	ui->combo_frame_score_type->setEnabled(false);
	ui->combo_score_filter->setEnabled(false);
	ui->spin_score_filter_threshold->setEnabled(false);
	ui->combo_layer_image_left->setEnabled(false);
	ui->combo_layer_image_middle->setEnabled(false);
	ui->combo_layer_image_right->setEnabled(false);
	ui->combo_lidar_return->setEnabled(false);
	ui->combo_bounding_box_type->setEnabled(false);
	ui->combo_field_of_view->setEnabled(false);

	ui->combo_layer_point_cloud->setCurrentIndex(
			static_cast<int>(layout_.point_cloud_viewer_layer));
	ui->combo_layer_image_left->setCurrentIndex(static_cast<int>(layout_.image_viewer_layer_left));
	ui->combo_layer_image_middle->setCurrentIndex(
			static_cast<int>(layout_.image_viewer_layer_middle));
	ui->combo_layer_image_right->setCurrentIndex(
			static_cast<int>(layout_.image_viewer_layer_right));
	ui->combo_lidar_return->setCurrentIndex(static_cast<int>(layout_.lidar_return));
	ui->combo_bounding_box_type->setCurrentIndex(static_cast<int>(layout_.bounding_box_type));
	ui->combo_field_of_view->setCurrentIndex(static_cast<int>(layout_.field_of_view));

	disconnectSignals();
}

void
Visualization::initializeUI()
{
	initialized_ = false;

	const auto &parameter = depth_clustering_->getParameter();
	auto folder_reader_range = depth_clustering_->getFolderReaderRange();

	if (!folder_reader_range)
	{
		std::cerr << "[ERROR]: Range image folder reader missing." << std::endl;
		return;
	}

	const auto &frame_paths_names_range = folder_reader_range->GetAllFilePaths();

	if (frame_paths_names_range.empty())
	{
		std::cerr << "[ERROR]: Range image missing in \"" << dataset_path_ << "\"." << std::endl;
		return;
	}

	ui->slider_frame->setMaximum(frame_paths_names_range.size() - 1);
	ui->spin_frame->setMaximum(frame_paths_names_range.size() - 1);

	ui->slider_frame->setValue(ui->slider_frame->minimum());
	ui->spin_frame->setValue(ui->slider_frame->minimum());
	ui->spin_angle_ground_removal->setValue(parameter.angle_ground_removal.ToDegrees());
	ui->combo_size_smooth_window->setCurrentIndex((parameter.size_smooth_window - 5) / 2);
	ui->spin_clustering_score_threshold->setValue(parameter.score_clustering);

	switch (parameter.difference_type)
	{
	case DiffFactory::DiffType::ANGLES:
	{
		ui->spin_clustering_depth_threshold->setValue(parameter.angle_clustering.ToDegrees());
		break;
	}
	case DiffFactory::DiffType::ANGLES_PRECOMPUTED:
	{
		ui->spin_clustering_depth_threshold->setValue(parameter.angle_clustering.ToDegrees());
		break;
	}
	case DiffFactory::DiffType::LINE_DIST:
	{
		ui->spin_clustering_depth_threshold->setValue(parameter.distance_clustering);
		break;
	}
	case DiffFactory::DiffType::LINE_DIST_PRECOMPUTED:
	{
		ui->spin_clustering_depth_threshold->setValue(parameter.distance_clustering);
		break;
	}
	case DiffFactory::DiffType::SIMPLE:
	{
		ui->spin_clustering_depth_threshold->setValue(parameter.distance_clustering);
		break;
	}
	default:
	{
		ui->spin_clustering_depth_threshold->setValue(parameter.angle_clustering.ToDegrees());
		break;
	}
	}

	ui->combo_difference_type->setCurrentIndex(static_cast<int>(parameter.difference_type));
	ui->spin_size_cluster_min->setValue(parameter.size_cluster_min);
	ui->spin_size_cluster_max->setValue(parameter.size_cluster_max);

	ui->combo_layer_point_cloud->setCurrentIndex(
			static_cast<int>(layout_.point_cloud_viewer_layer));
	ui->combo_score_filter->setCurrentIndex(static_cast<int>(parameter.use_score_filter));
	ui->spin_score_filter_threshold->setValue(parameter.score_filter_threshold);
	ui->combo_layer_image_left->setCurrentIndex(static_cast<int>(layout_.image_viewer_layer_left));
	ui->combo_layer_image_middle->setCurrentIndex(
			static_cast<int>(layout_.image_viewer_layer_middle));
	ui->combo_layer_image_right->setCurrentIndex(
			static_cast<int>(layout_.image_viewer_layer_right));
	ui->combo_lidar_return->setCurrentIndex(static_cast<int>(layout_.lidar_return));
	ui->combo_bounding_box_type->setCurrentIndex(static_cast<int>(layout_.bounding_box_type));
	ui->combo_field_of_view->setCurrentIndex(static_cast<int>(layout_.field_of_view));

	if (ui->combo_field_of_view->currentIndex()
			== static_cast<int>(VisualizationLayout::FieldOfView::Default))
	{
		ui->viewer_point_cloud->resetViewFOVDefault();
	}
	else if (ui->combo_field_of_view->currentIndex()
			== static_cast<int>(VisualizationLayout::FieldOfView::Camera))
	{
		ui->viewer_point_cloud->resetViewFOVCamera();
	}

	setWindowTitle(QString::fromStdString(dataset_path_));

	onParameterUpdated();
	connectSignals();

	ui->button_play->setEnabled(true);
	ui->button_pause->setEnabled(false);
	ui->button_stop->setEnabled(true);
	ui->slider_frame->setEnabled(true);
	ui->spin_frame->setEnabled(true);
	ui->spin_angle_ground_removal->setEnabled(true);
	ui->combo_size_smooth_window->setEnabled(true);
	ui->spin_clustering_depth_threshold->setEnabled(true);
	ui->spin_clustering_score_threshold->setEnabled(true);
	ui->combo_difference_type->setEnabled(true);
	ui->spin_size_cluster_min->setEnabled(true);
	ui->spin_size_cluster_max->setEnabled(true);
	ui->button_global_configuration->setEnabled(true);
	ui->button_layout_configuration->setEnabled(true);
	ui->combo_layer_point_cloud->setEnabled(true);
	ui->combo_point_score_type->setEnabled(true);
	ui->combo_cluster_score_type->setEnabled(true);
	ui->combo_frame_score_type->setEnabled(true);
	ui->combo_score_filter->setEnabled(true);
	ui->spin_score_filter_threshold->setEnabled(true);
	ui->combo_layer_image_left->setEnabled(true);
	ui->combo_layer_image_middle->setEnabled(true);
	ui->combo_layer_image_right->setEnabled(true);
	ui->combo_lidar_return->setEnabled(true);
	ui->combo_bounding_box_type->setEnabled(true);
	ui->combo_field_of_view->setEnabled(true);

	initialized_ = true;
}

void
Visualization::connectSignals()
{
	disconnectSignals();

	connect(ui->button_play, SIGNAL(released()), this, SLOT(onPlay()));
	connect(ui->button_pause, SIGNAL(released()), this, SLOT(onPause()));
	connect(ui->button_stop, SIGNAL(released()), this, SLOT(onStop()));
	connect(ui->slider_frame, SIGNAL(valueChanged(int)), this, SLOT(onSliderMovedTo(int)));
	connect(ui->spin_angle_ground_removal, SIGNAL(valueChanged(double)), this,
			SLOT(onParameterUpdated()));
	connect(ui->combo_size_smooth_window, SIGNAL(activated(int)), this, SLOT(onParameterUpdated()));
	connect(ui->spin_clustering_depth_threshold, SIGNAL(valueChanged(double)), this,
			SLOT(onParameterUpdated()));
	connect(ui->spin_clustering_score_threshold, SIGNAL(valueChanged(double)), this,
			SLOT(onParameterUpdated()));
	connect(ui->combo_difference_type, SIGNAL(activated(int)), this,
			SLOT(onDifferenceTypeUpdated()));
	connect(ui->spin_size_cluster_min, SIGNAL(valueChanged(int)), this, SLOT(onParameterUpdated()));
	connect(ui->spin_size_cluster_max, SIGNAL(valueChanged(int)), this, SLOT(onParameterUpdated()));
	connect(ui->button_global_configuration, SIGNAL(released()), this,
			SLOT(onLoadGlobalConfiguration()));
	connect(ui->button_layout_configuration, SIGNAL(released()), this,
			SLOT(onLoadLayoutConfiguration()));
	connect(ui->combo_field_of_view, SIGNAL(activated(int)), this, SLOT(onFieldOfViewUpdated()));
	connect(ui->combo_layer_point_cloud, SIGNAL(activated(int)), this,
			SLOT(onLayerPointCloudUpdated()));
	connect(ui->combo_point_score_type, SIGNAL(activated(int)), this, SLOT(onParameterUpdated()));
	connect(ui->combo_cluster_score_type, SIGNAL(activated(int)), this, SLOT(onParameterUpdated()));
	connect(ui->combo_frame_score_type, SIGNAL(activated(int)), this, SLOT(onParameterUpdated()));
	connect(ui->combo_score_filter, SIGNAL(activated(int)), this, SLOT(onParameterUpdated()));
	connect(ui->spin_score_filter_threshold, SIGNAL(valueChanged(double)), this,
			SLOT(onParameterUpdated()));
	connect(ui->combo_layer_image_left, SIGNAL(activated(int)), this, SLOT(onLayerImageUpdated()));
	connect(ui->combo_layer_image_middle, SIGNAL(activated(int)), this,
			SLOT(onLayerImageUpdated()));
	connect(ui->combo_layer_image_right, SIGNAL(activated(int)), this, SLOT(onLayerImageUpdated()));
	connect(ui->combo_lidar_return, SIGNAL(activated(int)), this, SLOT(onParameterUpdated()));
	connect(ui->combo_bounding_box_type, SIGNAL(activated(int)), this, SLOT(onParameterUpdated()));
	connect(ui->splitter_viewer, SIGNAL(splitterMoved(int, int)), this,
			SLOT(onSplitterViewerMoved()));
}

void
Visualization::disconnectSignals()
{
	disconnect(ui->button_play, SIGNAL(released()), this, SLOT(onPlay()));
	disconnect(ui->button_pause, SIGNAL(released()), this, SLOT(onPause()));
	disconnect(ui->button_stop, SIGNAL(released()), this, SLOT(onStop()));
	disconnect(ui->slider_frame, SIGNAL(valueChanged(int)), this, SLOT(onSliderMovedTo(int)));
	disconnect(ui->spin_angle_ground_removal, SIGNAL(valueChanged(double)), this,
			SLOT(onParameterUpdated()));
	disconnect(ui->combo_size_smooth_window, SIGNAL(activated(int)), this,
			SLOT(onParameterUpdated()));
	disconnect(ui->spin_clustering_depth_threshold, SIGNAL(valueChanged(double)), this,
			SLOT(onParameterUpdated()));
	disconnect(ui->spin_clustering_score_threshold, SIGNAL(valueChanged(double)), this,
			SLOT(onParameterUpdated()));
	disconnect(ui->combo_difference_type, SIGNAL(activated(int)), this,
			SLOT(onDifferenceTypeUpdated()));
	disconnect(ui->spin_size_cluster_min, SIGNAL(valueChanged(int)), this,
			SLOT(onParameterUpdated()));
	disconnect(ui->spin_size_cluster_max, SIGNAL(valueChanged(int)), this,
			SLOT(onParameterUpdated()));
	disconnect(ui->button_global_configuration, SIGNAL(released()), this,
			SLOT(onLoadGlobalConfiguration()));
	disconnect(ui->button_layout_configuration, SIGNAL(released()), this,
			SLOT(onLoadLayoutConfiguration()));
	disconnect(ui->combo_field_of_view, SIGNAL(activated(int)), this, SLOT(onFieldOfViewUpdated()));
	disconnect(ui->combo_layer_point_cloud, SIGNAL(activated(int)), this,
			SLOT(onLayerPointCloudUpdated()));
	disconnect(ui->combo_point_score_type, SIGNAL(activated(int)), this,
			SLOT(onParameterUpdated()));
	disconnect(ui->combo_cluster_score_type, SIGNAL(activated(int)), this,
			SLOT(onParameterUpdated()));
	disconnect(ui->combo_frame_score_type, SIGNAL(activated(int)), this,
			SLOT(onParameterUpdated()));
	disconnect(ui->combo_score_filter, SIGNAL(activated(int)), this, SLOT(onParameterUpdated()));
	disconnect(ui->spin_score_filter_threshold, SIGNAL(valueChanged(double)), this,
			SLOT(onParameterUpdated()));
	disconnect(ui->combo_layer_image_left, SIGNAL(activated(int)), this,
			SLOT(onLayerImageUpdated()));
	disconnect(ui->combo_layer_image_middle, SIGNAL(activated(int)), this,
			SLOT(onLayerImageUpdated()));
	disconnect(ui->combo_layer_image_right, SIGNAL(activated(int)), this,
			SLOT(onLayerImageUpdated()));
	disconnect(ui->combo_lidar_return, SIGNAL(activated(int)), this, SLOT(onParameterUpdated()));
	disconnect(ui->combo_bounding_box_type, SIGNAL(activated(int)), this,
			SLOT(onParameterUpdated()));
	disconnect(ui->splitter_viewer, SIGNAL(splitterMoved(int, int)), this,
			SLOT(onSplitterViewerMoved()));
}
} // namespace visualization
