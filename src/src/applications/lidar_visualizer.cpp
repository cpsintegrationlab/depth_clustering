#include <QApplication>
#include <QMainWindow>

#include "visualization/drawables/drawable_cloud.h"
#include "visualization/drawables/drawable_cube.h"
#include "visualization/drawables/drawable_polygon3d.h"
#include "visualization/visualization.h"

using depth_clustering::DrawableCube;
using depth_clustering::DrawablePolygon3d;

std::shared_ptr<Viewer> viewer_point_cloud_ = nullptr;

void
nonInteractiveModeThread(int argc, char* argv[])
{
	QApplication application(argc, argv);

	viewer_point_cloud_ = std::make_shared<Viewer>();
	viewer_point_cloud_->setWindowTitle(QCoreApplication::arguments().at(0));
	viewer_point_cloud_->show();

	application.exec();
}

int
nonInteractiveMode(int argc, char* argv[])
{
	DepthClustering depth_clustering;
	std::string dataset_path = std::string(argv[2]);
	std::thread thread(nonInteractiveModeThread, argc, argv);

	while (!viewer_point_cloud_)
	{
	}

	viewer_point_cloud_->setWindowTitle(QString::fromStdString(dataset_path));
	depth_clustering.initializeForDataset(dataset_path);

	auto folder_reader = depth_clustering.getFolderReader();
	const auto &frame_paths_names = folder_reader->GetAllFilePaths();

	if (frame_paths_names.empty())
	{
		std::cerr << "[ERROR]: Empty folder at \"" << dataset_path << "\"." << std::endl;
		return -1;
	}

	for (const auto &frame_path_name : frame_paths_names)
	{
		depth_clustering.processOneFrameForDataset(frame_path_name);

		auto current_cloud = depth_clustering.getCurrentCloud();
		auto bounding_box = depth_clustering.getBoundingBox();
		const auto &parameter = depth_clustering.getParameter();

		viewer_point_cloud_->Clear();
		viewer_point_cloud_->AddDrawable(DrawableCloud::FromCloud(current_cloud));

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

				viewer_point_cloud_->AddDrawable(std::move(cube_drawable));
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

				viewer_point_cloud_->AddDrawable(std::move(polygon_drawable));
			}

			break;
		}
		case BoundingBox::Type::Flat:
		{
			std::cerr << "[ERROR]: Cannot display flat bounding box in lidar visualizer."
					<< std::endl;
			return -1;
		}
		default:
		{
			break;
		}
		}

		viewer_point_cloud_->update();
	}

	thread.join();

	return 0;
}

int
interactiveMode(int argc, char* argv[])
{
	QApplication application(argc, argv);
	Visualization visualization;

	visualization.show();

	return application.exec();
}

int
main(int argc, char* argv[])
{
	if (argc > 1)
	{
		if (std::string(argv[1]) == "-h")
		{
			std::cout << std::endl << "Usage:\t" << argv[0] << std::endl;
			std::cout << "\t" << argv[0] << " [dataset path]" << std::endl;
			std::cout << "\t" << argv[0] << " -ni [dataset path]" << std::endl << std::endl;
			return 0;
		}
		else if (std::string(argv[1]) == "-ni")
		{
			if (argc < 3)
			{
				std::cout << std::endl << "Usage:\t" << argv[0] << " -ni [dataset path]"
						<< std::endl << std::endl;
				return 0;
			}

			return nonInteractiveMode(argc, argv);
		}
	}

	return interactiveMode(argc, argv);
}
