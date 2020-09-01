/*
 * main.cpp
 *
 *  Created on: Apr 28, 2020
 *      Author: simonyu
 */

#include "depth_clustering.h"

int
main(int argc, char* argv[])
{
	std::string mode = "";
	std::string data_folder = "../../data/waymo/";

	if (argc > 1)
	{
		if (std::string(argv[1]) == "-h")
		{
			std::cout << std::endl << "usage: " << argv[0] << "[data folder] [mode]" << std::endl << std::endl;
			return 0;
		}

		data_folder = argv[1];
	}

	if (argc > 2)
	{
		mode = argv[2];
	}

	DepthClustering depth_clustering;
	const std::string data_type = ".tiff";
	ObjectPainter::OutlineType outline_type = ObjectPainter::OutlineType::kBox;

	if (mode == "polygon")
	{
		outline_type = ObjectPainter::OutlineType::kPolygon3d;
	}

	if (!depth_clustering.init_data(data_folder, data_type, outline_type))
	{
		std::cout << "Failed to initialize. Quit." << std::endl;
		return -1;
	}

	depth_clustering.process_data();

	if (mode == "polygon")
	{
		auto outputs_frame = depth_clustering.get_output_data_polygon();
		std::cout << std::endl << "Total processed frames: " << outputs_frame.size() << "." << std::endl;
	}
	else
	{
		auto outputs_frame = depth_clustering.get_output_data_box();
		std::cout << std::endl << "Total processed frames: " << outputs_frame.size() << "." << std::endl;
	}

	depth_clustering.finish();

	return 0;
}
