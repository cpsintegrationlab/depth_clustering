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
		QWidget(parent), ui(new Ui::Visualization), play_(false)
{
	ui->setupUi(this);
	ui->button_play->setEnabled(false);
	ui->button_pause->setEnabled(false);
	ui->button_stop->setEnabled(false);
	ui->slider_frame->setEnabled(false);
	ui->spin_frame->setEnabled(false);
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
	connect(ui->spin_size_smooth_window, SIGNAL(valueChanged(int)), this,
			SLOT(onParameterUpdated()));
	connect(ui->spin_angle_clustering, SIGNAL(valueChanged(double)), this,
			SLOT(onParameterUpdated()));
	connect(ui->combo_difference_type, SIGNAL(activated(int)), this, SLOT(onParameterUpdated()));
	connect(ui->spin_size_cluster_min, SIGNAL(valueChanged(int)), this, SLOT(onParameterUpdated()));
	connect(ui->spin_size_cluster_max, SIGNAL(valueChanged(int)), this, SLOT(onParameterUpdated()));
	connect(ui->combo_bounding_box_type, SIGNAL(activated(int)), this, SLOT(onParameterUpdated()));

	depth_clustering_ = std::unique_ptr<DepthClustering>(new DepthClustering());
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

void
Visualization::onOpen()
{
	play_ = false;
	dataset_path_ = QFileDialog::getExistingDirectory(this).toStdString();

	if (!depth_clustering_)
	{
		std::cerr << "[ERROR]: API missing." << std::endl;
		return;
	}

	depth_clustering_->initializeForDataset(dataset_path_);

	auto folder_reader = depth_clustering_->getFolderReader();
	const auto &parameter = depth_clustering_->getParameter();
	const auto &frame_paths_names = folder_reader->GetAllFilePaths();

	if (frame_paths_names.empty())
	{
		std::cerr << "[ERROR]: Empty folder at \"" << dataset_path_ << "\"." << std::endl;
		return;
	}

	depth_clustering_->getClusterer()->SetLabelImageClient(this);

	ui->slider_frame->setMaximum(frame_paths_names.size() - 1);
	ui->spin_frame->setMaximum(frame_paths_names.size() - 1);
	ui->slider_frame->setValue(0);

	ui->button_play->setEnabled(true);
	ui->button_stop->setEnabled(true);
	ui->slider_frame->setEnabled(true);
	ui->spin_frame->setEnabled(true);

	ui->spin_size_smooth_window->setValue(parameter.size_smooth_window);
	ui->spin_angle_ground_removal->setValue(parameter.angle_ground_removal.ToDegrees());
	ui->spin_angle_clustering->setValue(parameter.angle_clustering.ToDegrees());
	ui->spin_size_cluster_min->setValue(parameter.size_cluster_min);
	ui->spin_size_cluster_max->setValue(parameter.size_cluster_max);
	ui->combo_difference_type->setCurrentIndex(static_cast<int>(parameter.difference_type));

	onSliderMovedTo(ui->slider_frame->value());

	std::cout << "[INFO]: Opened dataset at \"" << dataset_path_ << "\"." << std::endl;
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
		play_ = true;
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
	}
}

void
Visualization::onStop()
{
	onPause();

	ui->slider_frame->setValue(0);
	ui->viewer_point_cloud->update();
	QApplication::processEvents();
}

void
Visualization::onSliderMovedTo(int frame_number)
{
	Timer timer;
	auto folder_reader = depth_clustering_->getFolderReader();
	const auto &frame_paths_names = folder_reader->GetAllFilePaths();

	if (frame_paths_names.empty())
	{
		std::cerr << "[ERROR]: Empty folder at \"" << dataset_path_ << "\"." << std::endl;
		return;
	}

	std::cout << "[INFO]: Visualizing frame \"" << frame_paths_names[frame_number] << "\"."
			<< std::endl;

	const auto &frame_path_name = frame_paths_names[frame_number];
	setWindowTitle(QString::fromStdString(frame_path_name));

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

	parameter.angle_clustering = Radians::FromDegrees(ui->spin_angle_clustering->value());
	parameter.angle_ground_removal = Radians::FromDegrees(ui->spin_angle_ground_removal->value());
	parameter.size_cluster_min = ui->spin_size_cluster_min->value();
	parameter.size_cluster_max = ui->spin_size_cluster_max->value();
	parameter.size_smooth_window = ui->spin_size_smooth_window->value();

	BoundingBox::Type bounding_box_type = BoundingBox::Type::Cube;

	switch (ui->combo_bounding_box_type->currentIndex())
	{
	case 0:
	{
		std::cout << "[INFO]: Bounding box type: cube." << std::endl;
		bounding_box_type = BoundingBox::Type::Cube;
		break;
	}
	case 1:
	{
		std::cout << "[INFO]: Bounding box type: polygon." << std::endl;
		bounding_box_type = BoundingBox::Type::Polygon;
		break;
	}
	default:
	{
		std::cout << "[INFO]: Bounding box type: cube." << std::endl;
		bounding_box_type = BoundingBox::Type::Cube;
		break;
	}
	}

	parameter.bounding_box_type = bounding_box_type;

	DiffFactory::DiffType difference_type = DiffFactory::DiffType::ANGLES_PRECOMPUTED;

	switch (ui->combo_difference_type->currentIndex())
	{
	case 0:
	{
		std::cout << "[INFO]: Difference type: angles." << std::endl;
		difference_type = DiffFactory::DiffType::ANGLES;
		break;
	}
	case 1:
	{
		std::cout << "[INFO]: Difference type: precomputed angles." << std::endl;
		difference_type = DiffFactory::DiffType::ANGLES_PRECOMPUTED;
		break;
	}
	case 2:
	{
		std::cout << "[INFO]: Difference type: line distance." << std::endl;
		difference_type = DiffFactory::DiffType::LINE_DIST;
		break;
	}
	case 3:
	{
		std::cout << "[INFO]: Difference type: precomputed line distance." << std::endl;
		difference_type = DiffFactory::DiffType::LINE_DIST_PRECOMPUTED;
		break;
	}
	case 4:
	{
		std::cout << "[INFO]: Difference type: simple." << std::endl;
		difference_type = DiffFactory::DiffType::SIMPLE;
		break;
	}
	default:
	{
		std::cout << "[INFO]: Difference type: precomputed angles." << std::endl;
		difference_type = DiffFactory::DiffType::ANGLES_PRECOMPUTED;
		break;
	}
	}

	parameter.difference_type = difference_type;

	depth_clustering_->setParameter(parameter);

	auto clusterer = depth_clustering_->getClusterer();

	clusterer->SetDiffType(difference_type);
	clusterer->SetLabelImageClient(this);

	this->onSliderMovedTo(ui->slider_frame->value());

	std::cout << "[INFO]: Updated parameters." << std::endl;
}

void
Visualization::updateViewerPointCloud()
{
	auto current_cloud = depth_clustering_->getCurrentCloud();
	auto bounding_box = depth_clustering_->getBoundingBox();
	const auto &parameter = depth_clustering_->getParameter();

	ui->viewer_point_cloud->Clear();
	ui->viewer_point_cloud->AddDrawable(DrawableCloud::FromCloud(current_cloud));

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
		std::cerr << "[ERROR]: Cannot display flat bounding box in lidar visualizer." << std::endl;
		break;
	}
	default:
	{
		break;
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
