/*
 * camera_visualizer.cpp
 *
 *  Created on: Sep 11, 2020
 *      Author: simonyu
 */

#include <boost/property_tree/json_parser.hpp>
#include <opencv2/highgui.hpp>

#include "api/api.h"

using boost::property_tree::json_parser::read_json;

std::shared_ptr<BoundingBox::Frame<BoundingBox::Flat>>
getGroundTruthFrameFlat(const boost::property_tree::ptree& ground_truth_tree,
		const std::string& frame_name)
{
	std::shared_ptr<BoundingBox::Frame<BoundingBox::Flat>> bounding_box_frame_flat;

	auto bounding_box_frame_flat_tree_optioal = ground_truth_tree.get_child_optional(
			boost::property_tree::ptree::path_type(frame_name, '/'));

	if (!bounding_box_frame_flat_tree_optioal)
	{
		return bounding_box_frame_flat;
	}

	bounding_box_frame_flat = std::make_shared<BoundingBox::Frame<BoundingBox::Flat>>();
	auto bounding_box_frame_flat_tree = *bounding_box_frame_flat_tree_optioal;

	for (const auto &bounding_box_flat_array_pair : bounding_box_frame_flat_tree)
	{
		std::vector<std::string> bounding_box_flat_values;
		Eigen::Vector2i corner_upper_left;
		Eigen::Vector2i corner_lower_right;
		float depth;
		std::string id;

		for (const auto &bounding_box_flat_value_pair : bounding_box_flat_array_pair.second)
		{
			bounding_box_flat_values.push_back(
					bounding_box_flat_value_pair.second.get_value<std::string>());
		}

		corner_upper_left << std::stoi(bounding_box_flat_values[0]), std::stoi(
				bounding_box_flat_values[1]);
		corner_lower_right << std::stoi(bounding_box_flat_values[2]), std::stoi(
				bounding_box_flat_values[3]);
		depth = std::stof(bounding_box_flat_values[4]);
		id = bounding_box_flat_values[5];

		bounding_box_frame_flat->push_back(
				std::make_tuple(corner_upper_left, corner_lower_right, depth, id));
	}

	return bounding_box_frame_flat;
}

int
main(int argc, char* argv[])
{
	std::string dataset_path = "../../../data/segment-2506799708748258165_6455_000_6475_000/";
	int display_time = 0;

	if (argc > 1)
	{
		if (std::string(argv[1]) == "-h")
		{
			std::cout << std::endl << "Usage: " << argv[0] << " [dataset path] [display time]"
					<< std::endl << std::endl;
			return 0;
		}

		dataset_path = argv[1];
	}

	if (argc > 2)
	{
		display_time = std::stoi(std::string(argv[2]));
	}

	DepthClustering depth_clustering;
	boost::property_tree::ptree ground_truth_tree;

	if (!depth_clustering.initializeForDataset(dataset_path))
	{
		std::cout << "[ERROR]: Failed to initialize for dataset. Quit." << std::endl;
		return -1;
	}

	boost::property_tree::read_json(
			depth_clustering.getDatasetPath()
					+ depth_clustering.getParameter().ground_truth_flat_file_name,
			ground_truth_tree);

	cv::namedWindow(argv[0], cv::WINDOW_AUTOSIZE);

	while (1)
	{
		const std::string lidar_frame_name = depth_clustering.processNextFrameForDataset();
		std::string camera_frame_name = lidar_frame_name;

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
		auto ground_truth_frame_flat = getGroundTruthFrameFlat(ground_truth_tree, lidar_frame_name);

		if (bounding_box_frame_flat)
		{
			for (const auto &bounding_box_flat : *bounding_box_frame_flat)
			{
				cv::Point2i corner_upper_left(std::get<0>(bounding_box_flat).x(),
						std::get<0>(bounding_box_flat).y());
				cv::Point2i corner_lower_right(std::get<1>(bounding_box_flat).x(),
						std::get<1>(bounding_box_flat).y());
				cv::Point2i text_depth(std::get<0>(bounding_box_flat).x(),
						std::get<0>(bounding_box_flat).y() - 10);
				cv::rectangle(camera_frame, corner_upper_left, corner_lower_right,
						cv::Scalar(255, 0, 0), 2);
				cv::putText(camera_frame,
						"depth: " + std::to_string(std::get<2>(bounding_box_flat)), text_depth,
						cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(255, 0, 0), 2);
			}
		}
		else
		{
			std::cout << "[WARN]: Flat bounding box frame missing." << std::endl;
		}

		if (ground_truth_frame_flat)
		{
			for (const auto &ground_truth_flat : *ground_truth_frame_flat)
			{
				cv::Point2i corner_upper_left(std::get<0>(ground_truth_flat).x(),
						std::get<0>(ground_truth_flat).y());
				cv::Point2i corner_lower_right(std::get<1>(ground_truth_flat).x(),
						std::get<1>(ground_truth_flat).y());
				cv::Point2i text_depth(std::get<0>(ground_truth_flat).x(),
						std::get<0>(ground_truth_flat).y() - 10);
				cv::rectangle(camera_frame, corner_upper_left, corner_lower_right,
						cv::Scalar(0, 0, 255), 2);
				cv::putText(camera_frame,
						"depth: " + std::to_string(std::get<2>(ground_truth_flat)), text_depth,
						cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 0, 255), 2);
			}
		}
		else
		{
			std::cout << "[WARN]: Flat ground truth frame missing." << std::endl;
		}

		cv::imshow(argv[0], camera_frame);
		cv::waitKey(display_time);
	}

	return 0;
}
