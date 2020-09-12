/*
 * camera_visualizer.cpp
 *
 *  Created on: Sep 11, 2020
 *      Author: simonyu
 */

#include "api/api.h"
#include "opencv2/highgui.hpp"

int
main(int argc, char* argv[])
{
	std::string dataset_path =
			"../../../data/segment-1022527355599519580_4866_960_4886_960_with_camera_labels/";

	if (argc > 1)
	{
		if (std::string(argv[1]) == "-h")
		{
			std::cout << std::endl << "Usage: " << argv[0] << " [dataset path]" << std::endl
					<< std::endl;
			return 0;
		}

		dataset_path = argv[1];
	}

	DepthClustering depth_clustering;

	if (!depth_clustering.initializeForDataset(dataset_path))
	{
		std::cout << "[ERROR]: Failed to initialize for dataset. Quit." << std::endl;
		return -1;
	}

	cv::namedWindow(argv[0], cv::WINDOW_AUTOSIZE);

	while (1)
	{
		std::string camera_frame_name = depth_clustering.processNextFrameForDataset();

		if (camera_frame_name.empty())
		{
			break;
		}

		camera_frame_name.replace(camera_frame_name.find("lidar"), std::string("lidar").size(),
				"camera");
		camera_frame_name.replace(camera_frame_name.find(".tiff"), std::string(".tiff").size(),
				".jpg");

		cv::Mat camera_frame = cv::imread(dataset_path + camera_frame_name, CV_LOAD_IMAGE_COLOR);

		auto bounding_box_frame_flat = depth_clustering.getBoundingBoxFrameFlat();

		if (bounding_box_frame_flat)
		{
			for (const auto &bounding_box_flat : *bounding_box_frame_flat)
			{
				cv::Point2i corner_upper_left(std::get<0>(bounding_box_flat).x(), std::get<0>(bounding_box_flat).y());
				cv::Point2i corner_lower_right(std::get<1>(bounding_box_flat).x(), std::get<1>(bounding_box_flat).y());
				cv::rectangle(camera_frame, corner_upper_left, corner_lower_right, cv::Scalar(255, 0, 0), 2);
			}
		}
		else
		{
			std::cout << "[WARN]: Flat bounding box frame missing." << std::endl;
		}

		cv::imshow(argv[0], camera_frame);
		cv::waitKey(1);
	}

	return 0;
}
