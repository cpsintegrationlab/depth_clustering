/*
 * test.cpp
 *
 *  Created on: Apr 28, 2020
 *      Author: simonyu
 */

#include "depth_clustering.h"

int
main()
{
	DepthClustering depth_clustering;
	std::queue<ObjectPainter::OutputBoxFrame> outputs_box_frame;
	std::string data_folder = "../../data/moosmann/velodyne_slam/scenario_1/";

	outputs_box_frame = depth_clustering.process_data_box(data_folder);

	std::cout << "size of outputs_box_frame: " << outputs_box_frame.size() << std::endl;
}
