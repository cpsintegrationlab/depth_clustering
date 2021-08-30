#include <QColor>
#include <QDebug>
#include <QFileDialog>
#include <QImage>
#include <QPixmap>
#include <QString>
#include <QUuid>
#include <vector>

#if PCL_FOUND
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#endif	// PCL_FOUND

#include "api/parameter.h"
#include "image_labelers/diff_helpers/diff_factory.h"
#include "utils/folder_reader.h"
#include "utils/timer.h"
#include "utils/velodyne_utils.h"
#include "visualization/drawables/drawable_cloud.h"
#include "visualization/drawables/drawable_cube.h"
#include "visualization/utils/utils.h"
#include "visualization/ui_visualization.h"
#include "visualization/visualization.h"

using depth_clustering::AbstractImageLabeler;
using depth_clustering::DiffFactory;
using depth_clustering::FolderReader;
using depth_clustering::MatFromDepthPng;
using depth_clustering::ReadKittiCloud;
using depth_clustering::ReadKittiCloudTxt;
using depth_clustering::time_utils::Timer;

Visualization::Visualization(QWidget* parent) :
		QWidget(parent), ui(new Ui::Visualization)
{
	ui->setupUi(this);
	ui->sldr_navigate_clouds->setEnabled(false);
	ui->spnbx_current_cloud->setEnabled(false);
	ui->gfx_projection_view->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	ui->gfx_projection_view->setCacheMode(QGraphicsView::CacheBackground);
	ui->gfx_projection_view->setRenderHints(
			QPainter::Antialiasing | QPainter::SmoothPixmapTransform);

	ui->gfx_labels->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	ui->gfx_labels->setCacheMode(QGraphicsView::CacheBackground);
	ui->gfx_labels->setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);

	setWindowTitle(QCoreApplication::arguments().at(0));

	viewer_ = ui->gl_widget;
	viewer_->installEventFilter(this);
	viewer_->setAutoFillBackground(true);

	connect(ui->btn_open_folder, SIGNAL(released()), this, SLOT(onOpenFolder()));
	connect(ui->sldr_navigate_clouds, SIGNAL(valueChanged(int)), this, SLOT(onSliderMovedTo(int)));
	connect(ui->btn_play, SIGNAL(released()), this, SLOT(onVisualizeAllFrames()));

	connect(ui->spnbx_min_cluster_size, SIGNAL(valueChanged(int)), this, SLOT(onParameterUpdate()));
	connect(ui->spnbx_max_cluster_size, SIGNAL(valueChanged(int)), this, SLOT(onParameterUpdate()));
	connect(ui->spnbx_ground_angle, SIGNAL(valueChanged(double)), this, SLOT(onParameterUpdate()));
	connect(ui->spnbx_separation_angle, SIGNAL(valueChanged(double)), this,
			SLOT(onParameterUpdate()));
	connect(ui->spnbx_smooth_window_size, SIGNAL(valueChanged(int)), this,
			SLOT(onParameterUpdate()));
	connect(ui->radio_show_segmentation, SIGNAL(toggled(bool)), this, SLOT(onParameterUpdate()));
	connect(ui->radio_show_angles, SIGNAL(toggled(bool)), this, SLOT(onParameterUpdate()));
	connect(ui->cmb_diff_type, SIGNAL(activated(int)), this, SLOT(onParameterUpdate()));

	depth_clustering_ = std::unique_ptr<DepthClustering>(new DepthClustering());
}

void
Visualization::OnNewObjectReceived(const cv::Mat& image, int client_id)
{
	QImage qimage;

	switch (image.type())
	{
	case cv::DataType<float>::type:
	{
		std::cout << "[INFO]: Received angle image." << std::endl;

		auto difference_type = depth_clustering_->getParameter().difference_type;
		auto projection_parameter = depth_clustering_->getProjectionParameter();
		auto difference_helper = DiffFactory::Build(difference_type, &image,
				projection_parameter.get());

		qimage = MatToQImage(difference_helper->Visualize());

		break;
	}
	case cv::DataType<uint16_t>::type:
	{
		std::cout << "[INFO]: Received segmentation image." << std::endl;

		qimage = MatToQImage(AbstractImageLabeler::LabelsToColor(image));
		break;
	}
	default:
	{
		std::cerr << "[ERROR]: Received unknown image." << std::endl;
		return;
	}
	}

	scene_labels_.reset(new QGraphicsScene);
	scene_labels_->addPixmap(QPixmap::fromImage(qimage));
	ui->gfx_labels->setScene(scene_labels_.get());
	ui->gfx_labels->fitInView(scene_labels_->itemsBoundingRect());
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
		ui->spnbx_current_cloud->setValue(ui->spnbx_current_cloud->value() + 1);
		break;
	}
	case Qt::Key_Left:
	{
		ui->spnbx_current_cloud->setValue(ui->spnbx_current_cloud->value() - 1);
		break;
	}
	}
}

void
Visualization::onOpenFolder()
{
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

	ui->sldr_navigate_clouds->setMaximum(frame_paths_names.size() - 1);
	ui->spnbx_current_cloud->setMaximum(frame_paths_names.size() - 1);
	ui->sldr_navigate_clouds->setValue(0);
	ui->sldr_navigate_clouds->setEnabled(true);
	ui->spnbx_current_cloud->setEnabled(true);

	ui->spnbx_smooth_window_size->setValue(parameter.size_smooth_window);
	ui->spnbx_ground_angle->setValue(parameter.angle_ground_removal.ToDegrees());
	ui->spnbx_separation_angle->setValue(parameter.angle_clustering.ToDegrees());
	ui->spnbx_min_cluster_size->setValue(parameter.size_cluster_min);
	ui->spnbx_max_cluster_size->setValue(parameter.size_cluster_max);
	ui->cmb_diff_type->setCurrentIndex(static_cast<int>(parameter.difference_type));

	setWindowTitle(QString::fromUtf8(parameter.dataset_name.c_str()));

	viewer_->update();

	std::cout << "[INFO]: Opened dataset at \"" << dataset_path_ << "\"." << std::endl;
}

void
Visualization::onVisualizeAllFrames()
{
	for (int i = ui->sldr_navigate_clouds->minimum(); i <= ui->sldr_navigate_clouds->maximum(); ++i)
	{
		ui->sldr_navigate_clouds->setValue(i);
		viewer_->update();
		QApplication::processEvents();
	}

	std::cout << "[INFO]: All frames visualized." << std::endl;
}

void
Visualization::onParameterUpdate()
{
	DepthClusteringParameter parameter = depth_clustering_->getParameter();

	parameter.size_smooth_window = ui->spnbx_smooth_window_size->value();
	parameter.angle_ground_removal = Radians::FromDegrees(ui->spnbx_ground_angle->value());
	parameter.angle_clustering = Radians::FromDegrees(ui->spnbx_separation_angle->value());
	parameter.size_cluster_min = ui->spnbx_min_cluster_size->value();
	parameter.size_cluster_max = ui->spnbx_max_cluster_size->value();

	depth_clustering_->setParameter(parameter);

	DiffFactory::DiffType diff_type = DiffFactory::DiffType::NONE;

	switch (ui->cmb_diff_type->currentIndex())
	{
	case 0:
	{
		diff_type = DiffFactory::DiffType::ANGLES;
		break;
	}
	case 1:
	{
		diff_type = DiffFactory::DiffType::ANGLES_PRECOMPUTED;
		break;
	}
	case 2:
	{
		diff_type = DiffFactory::DiffType::LINE_DIST;
		break;
	}
	case 3:
	{
		diff_type = DiffFactory::DiffType::LINE_DIST_PRECOMPUTED;
		break;
	}
	default:
	{
		diff_type = DiffFactory::DiffType::SIMPLE;
		break;
	}
	}

	auto clusterer = depth_clustering_->getClusterer();

	clusterer->SetDiffType(diff_type);
	clusterer->SetLabelImageClient(this);

	onSliderMovedTo(ui->sldr_navigate_clouds->value());

	std::cout << "[INFO]: Updated parameters." << std::endl;
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

	ui->lbl_cloud_name->setText(QString::fromStdString(frame_path_name));
	depth_clustering_->processOneFrameForDataset(frame_path_name);

	auto current_depth_image = depth_clustering_->getCurrentDepthImage();
	auto current_cloud = depth_clustering_->GetCurrentCloud();

	timer.start();

	if (ui->radio_show_angles->isChecked())
	{
		OnNewObjectReceived(current_depth_image);
		std::cout << "[INFO]: Displayed angle image in " << timer.measure(Timer::Units::Micro)
				<< "us." << std::endl;
	}

	QImage current_depth_qimage = MatToQImage(current_depth_image);
	scene_.reset(new QGraphicsScene);
	scene_->addPixmap(QPixmap::fromImage(current_depth_qimage));
	ui->gfx_projection_view->setScene(scene_.get());
	ui->gfx_projection_view->fitInView(scene_->itemsBoundingRect());

	std::cout << "[INFO]: Displayed depth image in " << timer.measure(Timer::Units::Micro) << "us."
			<< std::endl;

	viewer_->Clear();
	viewer_->AddDrawable(DrawableCloud::FromCloud(current_cloud));
	viewer_->update();

	std::cout << "[INFO]: Displayed point cloud in " << timer.measure(Timer::Units::Micro) << "us."
			<< std::endl;
}
