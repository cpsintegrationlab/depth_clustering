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
	std::string data_folder = "../../data/moosmann/velodyne_slam/scenario_1/";

	if (mode == "polygon")
	{
		std::vector<ObjectPainter::OutputPolygonFrame> outputs_polygon_frame;

		depth_clustering.init_data_polygon(data_folder);
		outputs_polygon_frame = depth_clustering.process_data_polygon();

		std::cout << std::endl;
		std::cout << "size of outputs_polygon_frame: " << outputs_polygon_frame.size() << std::endl;
	}
	else
	{
		std::vector<ObjectPainter::OutputBoxFrame> outputs_box_frame;

		depth_clustering.init_data_box(data_folder);
		outputs_box_frame = depth_clustering.process_data_box();

		std::cout << std::endl;
		std::cout << "size of outputs_box_frame: " << outputs_box_frame.size() << std::endl;
	}

	depth_clustering.finish();
}
