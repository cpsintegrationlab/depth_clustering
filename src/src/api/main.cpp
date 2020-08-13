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

	if (argc > 1)
	{
		mode = argv[1];
	}

	DepthClustering depth_clustering;
	const std::string data_folder = "../../data/waymo/";
	const std::string data_type = ".tiff";
	ObjectPainter::OutlineType outline_type = ObjectPainter::OutlineType::kBox;

	if (mode == "polygon")
	{
		std::vector<ObjectPainter::OutputPolygonFrame> outputs_polygon_frame;

		outline_type = ObjectPainter::OutlineType::kPolygon3d;

		if (!depth_clustering.init_data(data_folder, data_type, outline_type))
		{
			std::cout << "Failed to initialize. Quit." << std::endl;
			return -1;
		}

		outputs_polygon_frame = depth_clustering.process_data_polygon();

		std::cout << std::endl;
		std::cout << "Total processed frames: " << outputs_polygon_frame.size() << "." << std::endl;
	}
	else
	{
		std::vector<ObjectPainter::OutputBoxFrame> outputs_box_frame;

		if (!depth_clustering.init_data(data_folder, data_type, outline_type))
		{
			std::cout << "Failed to initialize. Quit." << std::endl;
			return -1;
		}

		outputs_box_frame = depth_clustering.process_data_box();

		std::cout << std::endl;
		std::cout << "Total processed frames: " << outputs_box_frame.size() << "." << std::endl;
	}

	depth_clustering.finish();

	return 0;
}
