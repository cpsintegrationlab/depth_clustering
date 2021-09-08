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
using depth_clustering::MatFromDepthPng;
using depth_clustering::ReadKittiCloud;
using depth_clustering::ReadKittiCloudTxt;
using depth_clustering::time_utils::Timer;

Visualization::Visualization(QWidget* parent) :
		QWidget(parent), ui(new Ui::Visualization), play_(false), show_bounding_box_(true)
{
	ui->setupUi(this);

	ui->frame_control->setFixedHeight(ui->frame_control->minimumHeight());
	ui->frame_parameter->setFixedHeight(ui->frame_parameter->minimumHeight());
	ui->viewer_image_difference->setFixedHeight(ui->viewer_image_difference->minimumHeight());
	ui->viewer_image_segmentation->setFixedHeight(ui->viewer_image_segmentation->minimumHeight());
	ui->viewer_image_depth->setFixedHeight(ui->viewer_image_depth->minimumHeight());

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

	ui->viewer_image_difference->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	ui->viewer_image_difference->setCacheMode(QGraphicsView::CacheBackground);
	ui->viewer_image_difference->setRenderHints(
			QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
	ui->viewer_image_segmentation->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	ui->viewer_image_segmentation->setCacheMode(QGraphicsView::CacheBackground);
	ui->viewer_image_segmentation->setRenderHints(
			QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
	ui->viewer_image_depth->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	ui->viewer_image_depth->setCacheMode(QGraphicsView::CacheBackground);
	ui->viewer_image_depth->setRenderHints(
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
	connect(ui->combo_bounding_box_type, SIGNAL(activated(int)), this, SLOT(onParameterUpdated()));

	scene_difference_.reset(new QGraphicsScene);
	scene_difference_->addPixmap(QPixmap::fromImage(QImage()));
	scene_segmentation_.reset(new QGraphicsScene);
	scene_segmentation_->addPixmap(QPixmap::fromImage(QImage()));
	scene_depth_.reset(new QGraphicsScene);
	scene_depth_->addPixmap(QPixmap::fromImage(QImage()));

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
	QImage qimage_segmentation = MatToQImage(
			AbstractImageLabeler::LabelsToColor(image_segmentation));

	scene_segmentation_.reset(new QGraphicsScene);
	scene_segmentation_->addPixmap(QPixmap::fromImage(qimage_segmentation));
	ui->viewer_image_segmentation->setScene(scene_segmentation_.get());
	ui->viewer_image_segmentation->fitInView(scene_segmentation_->itemsBoundingRect());
}

void
Visualization::OnNewObjectReceived(const Cloud& cloud_no_ground, int client_id)
{
	std::lock_guard<std::mutex> lock_guard(current_depth_image_mutex_);
	current_depth_image_ = depth_clustering_->getCurrentDepthImage();
	current_depth_image_no_ground_ = cloud_no_ground.projection_ptr()->depth_image();
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

	refreshViewer();
}

void
Visualization::resizeEvent(QResizeEvent* event)
{
	QWidget::resizeEvent(event);

	refreshViewer();
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
			std::cout << "[INFO]: Visualization stopped." << std::endl;
			return;
		}

		ui->slider_frame->setValue(i);
		ui->viewer_point_cloud->update();
		QApplication::processEvents();
	}

	onPause();

	std::cout << "[INFO]: Visualization completed." << std::endl;
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
	auto folder_reader_first_return_range = depth_clustering_->getFolderReaderFirstReturnRange();
	const auto &frame_paths_names = folder_reader_first_return_range->GetAllFilePaths();

	if (frame_paths_names.empty())
	{
		std::cerr << "[ERROR]: Empty folder at \"" << dataset_path_ << "\"." << std::endl;
		return;
	}

	std::cout << "[INFO]: Visualizing frame \"" << frame_paths_names[frame_number] << "\"."
			<< std::endl;

	const auto &frame_path_name = frame_paths_names[frame_number];

	depth_clustering_->processOneFrameForDataset(frame_path_name);

	timer.start();

	updateViewerImage();

	std::cout << "[INFO]: Updated difference and depth image viewers in "
			<< timer.measure(Timer::Units::Micro) << " us." << std::endl;

	updateViewerPointCloud();

	std::cout << "[INFO]: Updated point cloud viewer in " << timer.measure(Timer::Units::Micro)
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
Visualization::openDataset(const std::string& dataset_path)
{
	if (dataset_path == "")
	{
		std::cerr << "[ERROR]: Empty dataset path." << std::endl;
		return;
	}

	dataset_path_ = dataset_path;
	play_ = false;

	if (!depth_clustering_)
	{
		std::cerr << "[ERROR]: API missing." << std::endl;
		return;
	}

	depth_clustering_->initializeForDataset(dataset_path_);

	auto folder_reader_first_return_range = depth_clustering_->getFolderReaderFirstReturnRange();
	const auto &parameter = depth_clustering_->getParameter();
	const auto &frame_paths_names = folder_reader_first_return_range->GetAllFilePaths();

	if (frame_paths_names.empty())
	{
		std::cerr << "[ERROR]: Empty folder at \"" << dataset_path_ << "\"." << std::endl;
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
Visualization::separatePointCloud()
{
	cv::Mat current_depth_image;
	cv::Mat current_depth_image_no_ground;
	const ProjectionParams projection_parameter = *depth_clustering_->getProjectionParameter();

	{
		std::lock_guard<std::mutex> lock_guard(current_depth_image_mutex_);
		current_depth_image = current_depth_image_;
		current_depth_image_no_ground = current_depth_image_no_ground_;
	}

	auto current_depth_image_ground = current_depth_image;

	for (int row = 0; row < current_depth_image.rows; row++)
	{
		for (int col = 0; col < current_depth_image.cols; col++)
		{
			if (current_depth_image.at<float>(row, col)
					!= current_depth_image_no_ground.at<float>(row, col))
			{
				current_depth_image_ground.at<float>(row, col) = current_depth_image.at<float>(row,
						col);
			}
			else
			{
				current_depth_image_ground.at<float>(row, col) = 0.0;
			}
		}
	}

	auto cloud_ground = Cloud::FromImage(current_depth_image_ground, projection_parameter);
	auto cloud_no_ground = Cloud::FromImage(current_depth_image_no_ground, projection_parameter);

	return std::make_pair(cloud_ground, cloud_no_ground);
}

void
Visualization::updateViewerPointCloud()
{
	auto current_cloud = depth_clustering_->getCurrentCloud();
	auto bounding_box = depth_clustering_->getBoundingBox();
	const auto &parameter = depth_clustering_->getParameter();

	const auto cloud_separated = separatePointCloud();
	const auto cloud_ground = cloud_separated.first;
	const auto cloud_no_ground = cloud_separated.second;

	ui->viewer_point_cloud->Clear();
	ui->viewer_point_cloud->AddDrawable(
			DrawableCloud::FromCloud(cloud_ground, Eigen::Vector3f(255, 0, 0)));
	ui->viewer_point_cloud->AddDrawable(
			DrawableCloud::FromCloud(cloud_no_ground, Eigen::Vector3f(255, 255, 255)));

	if (show_bounding_box_)
	{
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
Visualization::updateViewerImage()
{
	auto image_depth = depth_clustering_->getCurrentDepthImage();
	auto difference_type = depth_clustering_->getParameter().difference_type;
	auto projection_parameter = depth_clustering_->getProjectionParameter();
	auto difference_helper = DiffFactory::Build(difference_type, &image_depth,
			projection_parameter.get());
	QImage qimage_difference = MatToQImage(difference_helper->Visualize());
	QImage qimage_depth = MatToQImage(image_depth);

	scene_difference_.reset(new QGraphicsScene);
	scene_difference_->addPixmap(QPixmap::fromImage(qimage_difference));
	ui->viewer_image_difference->setScene(scene_difference_.get());
	ui->viewer_image_difference->fitInView(scene_difference_->itemsBoundingRect());

	scene_depth_.reset(new QGraphicsScene);
	scene_depth_->addPixmap(QPixmap::fromImage(qimage_depth));
	ui->viewer_image_depth->setScene(scene_depth_.get());
	ui->viewer_image_depth->fitInView(scene_depth_->itemsBoundingRect());
}

void
Visualization::refreshViewer()
{
	ui->viewer_point_cloud->update();
	ui->viewer_image_difference->setScene(scene_difference_.get());
	ui->viewer_image_difference->fitInView(scene_difference_->itemsBoundingRect());
	ui->viewer_image_segmentation->setScene(scene_segmentation_.get());
	ui->viewer_image_segmentation->fitInView(scene_segmentation_->itemsBoundingRect());
	ui->viewer_image_depth->setScene(scene_depth_.get());
	ui->viewer_image_depth->fitInView(scene_depth_->itemsBoundingRect());
}
