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
	std::string dataset_path =
			"../../../data/segment-1022527355599519580_4866_960_4886_960_with_camera_labels/";
	const std::string dataset_file_type = ".tiff";
	std::string bounding_box_type_string = "cube";
	DepthClustering::Parameter depth_clustering_parameter;

	if (argc > 1)
	{
		if (std::string(argv[1]) == "-h")
		{
			std::cout << std::endl << "Usage: " << argv[0] << " [dataset path] [bounding box type]"
					<< std::endl << std::endl;
			return 0;
		}

		dataset_path = argv[1];
	}

	if (argc > 2)
	{
		bounding_box_type_string = argv[2];
	}

	if (bounding_box_type_string == "cube")
	{
		depth_clustering_parameter.bounding_box_type = BoundingBox::Type::Cube;
	}
	else if (bounding_box_type_string == "polygon")
	{
		depth_clustering_parameter.bounding_box_type = BoundingBox::Type::Polygon;
	}

	DepthClustering depth_clustering(depth_clustering_parameter);

	if (!depth_clustering.initializeForDataset(dataset_path, dataset_file_type))
	{
		std::cout << "[ERROR]: Failed to initialize for dataset. Quit." << std::endl;
		return -1;
	}

	depth_clustering.processForDataset();
	depth_clustering.finish();

	return 0;
}
