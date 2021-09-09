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

#include "api/parameter.h"
#include "image_labelers/diff_helpers/diff_factory.h"
#include "utils/folder_reader.h"
#include "utils/timer.h"
#include "utils/velodyne_utils.h"
#include "visualization/drawables/drawable_cloud.h"
#include "visualization/drawables/drawable_cube.h"
#include "visualization/drawables/drawable_polygon3d.h"
#include "visualization/utils/utils.h"
#include "visualization/ui_visualization.h"
#include "visualization/visualization.h"

using depth_clustering::AbstractImageLabeler;
using depth_clustering::DiffFactory;
using depth_clustering::DrawableCube;
using depth_clustering::DrawablePolygon3d;
using depth_clustering::FolderReader;
using depth_clustering::ReadKittiCloud;
using depth_clustering::ReadKittiCloudTxt;
using depth_clustering::time_utils::Timer;

Visualization::Visualization(QWidget* parent) :
		QWidget(parent), ui(new Ui::Visualization), play_(false), show_bounding_box_(true), viewer_point_cloud_layer_index_(
				0), viewer_image_layer_index_top_(0), viewer_image_layer_index_middle_(1), viewer_image_layer_index_bottom_(
				2)
{
	ui->setupUi(this);

	ui->frame_controls->setFixedHeight(ui->frame_controls->minimumHeight());
	ui->frame_settings->setFixedHeight(ui->frame_settings->minimumHeight());
	ui->viewer_image_top->setFixedHeight(ui->viewer_image_top->minimumHeight());
	ui->viewer_image_middle->setFixedHeight(ui->viewer_image_middle->minimumHeight());
	ui->viewer_image_bottom->setFixedHeight(ui->viewer_image_bottom->minimumHeight());

	ui->button_play->setEnabled(false);
	ui->button_pause->setEnabled(false);
	ui->button_stop->setEnabled(false);
	ui->slider_frame->setEnabled(false);
	ui->spin_frame->setEnabled(false);
	ui->spin_angle_ground_removal->setEnabled(false);
	ui->combo_size_smooth_window->setEnabled(false);
	ui->spin_clustering_threshold->setEnabled(false);
	ui->combo_difference_type->setEnabled(false);
	ui->spin_size_cluster_min->setEnabled(false);
	ui->spin_size_cluster_max->setEnabled(false);
	ui->combo_bounding_box_type->setEnabled(false);

	ui->combo_layer_point_cloud->setCurrentIndex(viewer_point_cloud_layer_index_);
	ui->combo_layer_image_top->setCurrentIndex(viewer_image_layer_index_top_);
	ui->combo_layer_image_middle->setCurrentIndex(viewer_image_layer_index_middle_);
	ui->combo_layer_image_bottom->setCurrentIndex(viewer_image_layer_index_bottom_);

	ui->viewer_image_top->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	ui->viewer_image_top->setCacheMode(QGraphicsView::CacheBackground);
	ui->viewer_image_top->setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
	ui->viewer_image_middle->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	ui->viewer_image_middle->setCacheMode(QGraphicsView::CacheBackground);
	ui->viewer_image_middle->setRenderHints(
			QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
	ui->viewer_image_bottom->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	ui->viewer_image_bottom->setCacheMode(QGraphicsView::CacheBackground);
	ui->viewer_image_bottom->setRenderHints(
			QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
	ui->viewer_point_cloud->installEventFilter(this);
	ui->viewer_point_cloud->setAutoFillBackground(true);

	setWindowTitle(QCoreApplication::arguments().at(0));

	connect(ui->button_open, SIGNAL(released()), this, SLOT(onOpen()));
	connect(ui->button_play, SIGNAL(released()), this, SLOT(onPlay()));
	connect(ui->button_pause, SIGNAL(released()), this, SLOT(onPause()));
	connect(ui->button_stop, SIGNAL(released()), this, SLOT(onStop()));
	connect(ui->slider_frame, SIGNAL(valueChanged(int)), this, SLOT(onSliderMovedTo(int)));
	connect(ui->spin_angle_ground_removal, SIGNAL(valueChanged(double)), this,
			SLOT(onParameterUpdated()));
	connect(ui->combo_size_smooth_window, SIGNAL(activated(int)), this, SLOT(onParameterUpdated()));
	connect(ui->spin_clustering_threshold, SIGNAL(valueChanged(double)), this,
			SLOT(onParameterUpdated()));
	connect(ui->combo_difference_type, SIGNAL(activated(int)), this,
			SLOT(onDifferenceTypeUpdated()));
	connect(ui->spin_size_cluster_min, SIGNAL(valueChanged(int)), this, SLOT(onParameterUpdated()));
	connect(ui->spin_size_cluster_max, SIGNAL(valueChanged(int)), this, SLOT(onParameterUpdated()));
	connect(ui->combo_layer_point_cloud, SIGNAL(activated(int)), this,
			SLOT(onLayerPointCloudUpdated()));
	connect(ui->combo_layer_image_top, SIGNAL(activated(int)), this, SLOT(onLayerImageUpdated()));
	connect(ui->combo_layer_image_middle, SIGNAL(activated(int)), this,
			SLOT(onLayerImageUpdated()));
	connect(ui->combo_layer_image_bottom, SIGNAL(activated(int)), this,
			SLOT(onLayerImageUpdated()));
	connect(ui->combo_bounding_box_type, SIGNAL(activated(int)), this, SLOT(onParameterUpdated()));
	connect(ui->button_page_next, SIGNAL(released()), this, SLOT(onNextPage()));
	connect(ui->button_page_last, SIGNAL(released()), this, SLOT(onLastPage()));

	scene_empty_.reset(new QGraphicsScene);
	scene_empty_->addPixmap(QPixmap::fromImage(QImage()));
	scene_difference_.reset(new QGraphicsScene);
	scene_difference_->addPixmap(QPixmap::fromImage(QImage()));
	scene_segmentation_.reset(new QGraphicsScene);
	scene_segmentation_->addPixmap(QPixmap::fromImage(QImage()));
	scene_range_.reset(new QGraphicsScene);
	scene_range_->addPixmap(QPixmap::fromImage(QImage()));

	depth_clustering_ = std::unique_ptr<DepthClustering>(new DepthClustering());

	const auto &arguments = QCoreApplication::arguments();

	if (arguments.size() > 1)
	{
		openDataset(arguments.at(1).toStdString());
	}
}

void
Visualization::OnNewObjectReceived(const cv::Mat& image_segmentation, int client_id)
{
	if (viewer_image_layer_index_top_ == 1 || viewer_image_layer_index_middle_ == 1
			|| viewer_image_layer_index_bottom_ == 1)
	{
		QImage qimage_segmentation = MatToQImage(
				AbstractImageLabeler::LabelsToColor(image_segmentation));

		scene_segmentation_.reset(new QGraphicsScene);
		scene_segmentation_->addPixmap(QPixmap::fromImage(qimage_segmentation));

		updateViewerImage();
	}
}

void
Visualization::OnNewObjectReceived(const Cloud& cloud_no_ground, int client_id)
{
	std::lock_guard<std::mutex> lock_guard(image_range_mutex_);
	image_range_ = depth_clustering_->getImageRange();
	image_range_no_ground_ = cloud_no_ground.projection_ptr()->depth_image();
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

	ui->viewer_point_cloud->update();

	updateViewerImage();
}

void
Visualization::resizeEvent(QResizeEvent* event)
{
	QWidget::resizeEvent(event);

	ui->viewer_point_cloud->update();

	updateViewerImage();
}

void
Visualization::onOpen()
{
	openDataset(QFileDialog::getExistingDirectory(this).toStdString());
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
	Timer timer;
	auto folder_reader_range = depth_clustering_->getFolderReaderRange();
	const auto &frame_paths_names = folder_reader_range->GetAllFilePaths();

	if (frame_paths_names.empty())
	{
		std::cerr << "[ERROR]: Invalid dataset path \"" << dataset_path_ << "\"." << std::endl;
		return;
	}

	std::cout << std::endl << "[INFO]: Visualizing frame \"" << frame_paths_names[frame_number]
			<< "\"." << std::endl;

	const auto &frame_path_name = frame_paths_names[frame_number];

	depth_clustering_->processOneRangeFrameForDataset(frame_path_name);

	timer.start();

	updateViewerImageScene();

	std::cout << "[INFO]: Updated image viewers: " << timer.measure(Timer::Units::Micro) << " us."
			<< std::endl;

	updateViewerPointCloud();

	std::cout << "[INFO]: Updated point cloud viewer: " << timer.measure(Timer::Units::Micro)
			<< " us." << std::endl;
}

void
Visualization::onParameterUpdated()
{
	DepthClusteringParameter parameter = depth_clustering_->getParameter();

	parameter.angle_ground_removal = Radians::FromDegrees(ui->spin_angle_ground_removal->value());
	parameter.size_cluster_min = ui->spin_size_cluster_min->value();
	parameter.size_cluster_max = ui->spin_size_cluster_max->value();
	parameter.size_smooth_window = ui->combo_size_smooth_window->currentIndex() * 2 + 5;

	BoundingBox::Type bounding_box_type = BoundingBox::Type::Cube;

	switch (ui->combo_bounding_box_type->currentIndex())
	{
	case 0:
	{
		std::cout << "[INFO]: Bounding box type: cube." << std::endl;
		bounding_box_type = BoundingBox::Type::Cube;
		show_bounding_box_ = true;
		break;
	}
	case 1:
	{
		std::cout << "[INFO]: Bounding box type: polygon." << std::endl;
		bounding_box_type = BoundingBox::Type::Polygon;
		show_bounding_box_ = true;
		break;
	}
	case 2:
	{
		std::cout << "[INFO]: Bounding box type: none." << std::endl;
		bounding_box_type = parameter.bounding_box_type;
		show_bounding_box_ = false;
		break;
	}
	default:
	{
		std::cout << "[INFO]: Bounding box type: cube." << std::endl;
		bounding_box_type = BoundingBox::Type::Cube;
		show_bounding_box_ = true;
		break;
	}
	}

	parameter.bounding_box_type = bounding_box_type;

	switch (ui->combo_difference_type->currentIndex())
	{
	case 0:
	{
		parameter.angle_clustering = Radians::FromDegrees(ui->spin_clustering_threshold->value());
		break;
	}
	case 1:
	{
		parameter.angle_clustering = Radians::FromDegrees(ui->spin_clustering_threshold->value());
		break;
	}
	case 2:
	{
		parameter.distance_clustering = ui->spin_clustering_threshold->value();
		break;
	}
	case 3:
	{
		parameter.distance_clustering = ui->spin_clustering_threshold->value();
		break;
	}
	case 4:
	{
		parameter.distance_clustering = ui->spin_clustering_threshold->value();
		break;
	}
	default:
	{
		parameter.angle_clustering = Radians::FromDegrees(ui->spin_clustering_threshold->value());
		break;
	}
	}

	depth_clustering_->setParameter(parameter);

	depth_clustering_->getDepthGroundRemover()->AddClient(this);
	depth_clustering_->getClusterer()->SetLabelImageClient(this);

	this->onSliderMovedTo(ui->slider_frame->value());

	std::cout << "[INFO]: Updated parameters." << std::endl;
}

void
Visualization::onLayerPointCloudUpdated()
{
	viewer_point_cloud_layer_index_ = ui->combo_layer_point_cloud->currentIndex();

	updateViewerPointCloud();
}

void
Visualization::onLayerImageUpdated()
{
	viewer_image_layer_index_top_ = ui->combo_layer_image_top->currentIndex();
	viewer_image_layer_index_middle_ = ui->combo_layer_image_middle->currentIndex();
	viewer_image_layer_index_bottom_ = ui->combo_layer_image_bottom->currentIndex();

	updateViewerImage();
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
		std::cout << "[INFO]: Difference type: angles." << std::endl;
		difference_type = DiffFactory::DiffType::ANGLES;
		ui->spin_clustering_threshold->setValue(parameter.angle_clustering.ToDegrees());
		break;
	}
	case 1:
	{
		std::cout << "[INFO]: Difference type: precomputed angles." << std::endl;
		difference_type = DiffFactory::DiffType::ANGLES_PRECOMPUTED;
		ui->spin_clustering_threshold->setValue(parameter.angle_clustering.ToDegrees());
		break;
	}
	case 2:
	{
		std::cout << "[INFO]: Difference type: line distance." << std::endl;
		difference_type = DiffFactory::DiffType::LINE_DIST;
		ui->spin_clustering_threshold->setValue(parameter.distance_clustering);
		break;
	}
	case 3:
	{
		std::cout << "[INFO]: Difference type: precomputed line distance." << std::endl;
		difference_type = DiffFactory::DiffType::LINE_DIST_PRECOMPUTED;
		ui->spin_clustering_threshold->setValue(parameter.distance_clustering);
		break;
	}
	case 4:
	{
		std::cout << "[INFO]: Difference type: simple distance." << std::endl;
		difference_type = DiffFactory::DiffType::SIMPLE;
		ui->spin_clustering_threshold->setValue(parameter.distance_clustering);
		break;
	}
	default:
	{
		std::cout << "[INFO]: Difference type: precomputed angles." << std::endl;
		difference_type = DiffFactory::DiffType::ANGLES_PRECOMPUTED;
		ui->spin_clustering_threshold->setValue(parameter.angle_clustering.ToDegrees());
		break;
	}
	}

	parameter.difference_type = difference_type;

	depth_clustering_->setParameter(parameter);

	depth_clustering_->getDepthGroundRemover()->AddClient(this);
	depth_clustering_->getClusterer()->SetDiffType(difference_type);
	depth_clustering_->getClusterer()->SetLabelImageClient(this);

	this->onSliderMovedTo(ui->slider_frame->value());

	std::cout << "[INFO]: Updated difference type." << std::endl;
}

void
Visualization::onNextPage()
{
	ui->stacked_settings->setCurrentIndex(1);
}

void
Visualization::onLastPage()
{
	ui->stacked_settings->setCurrentIndex(0);
}

void
Visualization::openDataset(const std::string& dataset_path)
{
	if (dataset_path == "")
	{
		std::cerr << "[ERROR]: Empty dataset path." << std::endl;
		return;
	}

	dataset_path_ = dataset_path;
	play_ = false;
	viewer_point_cloud_layer_index_ = 0;
	viewer_image_layer_index_top_ = 0;
	viewer_image_layer_index_middle_ = 1;
	viewer_image_layer_index_bottom_ = 2;

	if (!depth_clustering_)
	{
		std::cerr << "[ERROR]: API missing." << std::endl;
		return;
	}

	depth_clustering_->initializeForDataset(dataset_path_);

	auto folder_reader_range = depth_clustering_->getFolderReaderRange();
	const auto &parameter = depth_clustering_->getParameter();
	const auto &frame_paths_names = folder_reader_range->GetAllFilePaths();

	if (frame_paths_names.empty())
	{
		std::cerr << "[ERROR]: Invalid dataset path \"" << dataset_path_ << "\"." << std::endl;
		return;
	}

	depth_clustering_->getDepthGroundRemover()->AddClient(this);
	depth_clustering_->getClusterer()->SetLabelImageClient(this);

	ui->slider_frame->setMaximum(frame_paths_names.size() - 1);
	ui->spin_frame->setMaximum(frame_paths_names.size() - 1);
	ui->slider_frame->setValue(ui->slider_frame->minimum());

	ui->button_play->setEnabled(true);
	ui->button_pause->setEnabled(false);
	ui->button_stop->setEnabled(true);
	ui->slider_frame->setEnabled(true);
	ui->spin_frame->setEnabled(true);
	ui->spin_angle_ground_removal->setEnabled(true);
	ui->combo_size_smooth_window->setEnabled(true);
	ui->spin_clustering_threshold->setEnabled(true);
	ui->combo_difference_type->setEnabled(true);
	ui->spin_size_cluster_min->setEnabled(true);
	ui->spin_size_cluster_max->setEnabled(true);
	ui->combo_bounding_box_type->setEnabled(true);

	ui->combo_size_smooth_window->setCurrentIndex((parameter.size_smooth_window - 5) / 2);
	ui->spin_angle_ground_removal->setValue(parameter.angle_ground_removal.ToDegrees());
	ui->spin_size_cluster_min->setValue(parameter.size_cluster_min);
	ui->spin_size_cluster_max->setValue(parameter.size_cluster_max);

	ui->combo_layer_point_cloud->setCurrentIndex(viewer_point_cloud_layer_index_);
	ui->combo_layer_image_top->setCurrentIndex(viewer_image_layer_index_top_);
	ui->combo_layer_image_middle->setCurrentIndex(viewer_image_layer_index_middle_);
	ui->combo_layer_image_bottom->setCurrentIndex(viewer_image_layer_index_bottom_);

	ui->combo_difference_type->setCurrentIndex(static_cast<int>(parameter.difference_type));

	switch (parameter.difference_type)
	{
	case DiffFactory::DiffType::ANGLES:
	{
		ui->spin_clustering_threshold->setValue(parameter.angle_clustering.ToDegrees());
		break;
	}
	case DiffFactory::DiffType::ANGLES_PRECOMPUTED:
	{
		ui->spin_clustering_threshold->setValue(parameter.angle_clustering.ToDegrees());
		break;
	}
	case DiffFactory::DiffType::LINE_DIST:
	{
		ui->spin_clustering_threshold->setValue(parameter.distance_clustering);
		break;
	}
	case DiffFactory::DiffType::LINE_DIST_PRECOMPUTED:
	{
		ui->spin_clustering_threshold->setValue(parameter.distance_clustering);
		break;
	}
	case DiffFactory::DiffType::SIMPLE:
	{
		ui->spin_clustering_threshold->setValue(parameter.distance_clustering);
		break;
	}
	default:
	{
		ui->spin_clustering_threshold->setValue(parameter.angle_clustering.ToDegrees());
		break;
	}
	}

	ui->combo_bounding_box_type->setCurrentIndex(static_cast<int>(parameter.bounding_box_type));

	setWindowTitle(QString::fromStdString(dataset_path_));

	onSliderMovedTo(ui->slider_frame->value());

	std::cout << "[INFO]: Opened dataset at \"" << dataset_path_ << "\"." << std::endl;
}

std::pair<Cloud::ConstPtr, Cloud::ConstPtr>
Visualization::extractGroundPointCloud()
{
	cv::Mat image_range;
	cv::Mat image_range_no_ground;
	const ProjectionParams projection_parameter = *depth_clustering_->getProjectionParameter();

	{
		std::lock_guard<std::mutex> lock_guard(image_range_mutex_);
		image_range = image_range_;
		image_range_no_ground = image_range_no_ground_;
	}

	auto image_range_ground = image_range;

	for (int row = 0; row < image_range.rows; row++)
	{
		for (int col = 0; col < image_range.cols; col++)
		{
			if (image_range.at<float>(row, col) != image_range_no_ground.at<float>(row, col))
			{
				image_range_ground.at<float>(row, col) = image_range.at<float>(row, col);
			}
			else
			{
				image_range_ground.at<float>(row, col) = 0.0;
			}
		}
	}

	auto cloud_ground = Cloud::FromImage(image_range_ground, projection_parameter);
	auto cloud_no_ground = Cloud::FromImage(image_range_no_ground, projection_parameter);

	return std::make_pair(cloud_ground, cloud_no_ground);
}

void
Visualization::updateViewerPointCloud()
{
	ui->viewer_point_cloud->Clear();

	if (viewer_point_cloud_layer_index_ == 0)
	{
		const auto cloud_separated = extractGroundPointCloud();
		const auto cloud_ground = cloud_separated.first;
		const auto cloud_no_ground = cloud_separated.second;

		ui->viewer_point_cloud->AddDrawable(
				DrawableCloud::FromCloud(cloud_ground, Eigen::Vector3f(255, 0, 0)));
		ui->viewer_point_cloud->AddDrawable(
				DrawableCloud::FromCloud(cloud_no_ground, Eigen::Vector3f(255, 255, 255)));
	}
	else
	{
		ui->viewer_point_cloud->AddDrawable(
				DrawableCloud::FromCloud(depth_clustering_->getCloud()));
	}

	if (show_bounding_box_)
	{
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

				auto cube_drawable = DrawableCube::Create(center, extent);

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

				auto polygon_drawable = DrawablePolygon3d::Create(hull, diff_z);

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
Visualization::updateViewerImageScene()
{
	if (viewer_image_layer_index_top_ == 0 || viewer_image_layer_index_middle_ == 0
			|| viewer_image_layer_index_bottom_ == 0)
	{
		auto image_range = depth_clustering_->getImageRange();
		auto difference_type = depth_clustering_->getParameter().difference_type;
		auto projection_parameter = depth_clustering_->getProjectionParameter();
		auto difference_helper = DiffFactory::Build(difference_type, &image_range,
				projection_parameter.get());
		QImage qimage_difference = MatToQImage(difference_helper->Visualize());

		scene_difference_.reset(new QGraphicsScene);
		scene_difference_->addPixmap(QPixmap::fromImage(qimage_difference));

		updateViewerImage();
	}

	if (viewer_image_layer_index_top_ == 2 || viewer_image_layer_index_middle_ == 2
			|| viewer_image_layer_index_bottom_ == 2)
	{
		auto image_range = depth_clustering_->getImageRange();
		QImage qimage_range = MatToQImage(image_range);

		scene_range_.reset(new QGraphicsScene);
		scene_range_->addPixmap(QPixmap::fromImage(qimage_range));

		updateViewerImage();
	}
}

void
Visualization::updateViewerImage()
{
	switch (viewer_image_layer_index_top_)
	{
	case 0:
	{
		ui->viewer_image_top->setScene(scene_difference_.get());
		ui->viewer_image_top->fitInView(scene_difference_->itemsBoundingRect());
		break;
	}
	case 1:
	{
		ui->viewer_image_top->setScene(scene_segmentation_.get());
		ui->viewer_image_top->fitInView(scene_segmentation_->itemsBoundingRect());
		break;
	}
	case 2:
	{
		ui->viewer_image_top->setScene(scene_range_.get());
		ui->viewer_image_top->fitInView(scene_range_->itemsBoundingRect());
		break;
	}
	case 3:
	{
		// TODO implement this
		break;
	}
	case 4:
	{
		// TODO implement this
		break;
	}
	default:
	{
		ui->viewer_image_top->setScene(scene_empty_.get());
		ui->viewer_image_top->fitInView(scene_empty_->itemsBoundingRect());
		break;
	}
	}

	switch (viewer_image_layer_index_middle_)
	{
	case 0:
	{
		ui->viewer_image_middle->setScene(scene_difference_.get());
		ui->viewer_image_middle->fitInView(scene_difference_->itemsBoundingRect());
		break;
	}
	case 1:
	{
		ui->viewer_image_middle->setScene(scene_segmentation_.get());
		ui->viewer_image_middle->fitInView(scene_segmentation_->itemsBoundingRect());
		break;
	}
	case 2:
	{
		ui->viewer_image_middle->setScene(scene_range_.get());
		ui->viewer_image_middle->fitInView(scene_range_->itemsBoundingRect());
		break;
	}
	case 3:
	{
		// TODO implement this
		break;
	}
	case 4:
	{
		// TODO implement this
		break;
	}
	default:
	{
		ui->viewer_image_middle->setScene(scene_empty_.get());
		ui->viewer_image_middle->fitInView(scene_empty_->itemsBoundingRect());
		break;
	}
	}

	switch (viewer_image_layer_index_bottom_)
	{
	case 0:
	{
		ui->viewer_image_bottom->setScene(scene_difference_.get());
		ui->viewer_image_bottom->fitInView(scene_difference_->itemsBoundingRect());
		break;
	}
	case 1:
	{
		ui->viewer_image_bottom->setScene(scene_segmentation_.get());
		ui->viewer_image_bottom->fitInView(scene_segmentation_->itemsBoundingRect());
		break;
	}
	case 2:
	{
		ui->viewer_image_bottom->setScene(scene_range_.get());
		ui->viewer_image_bottom->fitInView(scene_range_->itemsBoundingRect());
		break;
	}
	case 3:
	{
		// TODO implement this
		break;
	}
	case 4:
	{
		// TODO implement this
		break;
	}
	default:
	{
		ui->viewer_image_bottom->setScene(scene_empty_.get());
		ui->viewer_image_bottom->fitInView(scene_empty_->itemsBoundingRect());
		break;
	}
	}
}
