/*
 * main.cpp
 *
 *  Created on: Apr 28, 2020
 *      Author: simonyu
 */

#include "api/depth_clustering.h"

int
main(int argc, char* argv[])
{
	std::string dataset_path =
			"../../../data/segment-1022527355599519580_4866_960_4886_960_with_camera_labels/";
	const std::string dataset_file_type = ".tiff";

	if (argc > 1)
	{
		if (std::string(argv[1]) == "-h")
		{
			std::cout << std::endl << "Usage: " << argv[0] << " [dataset path]"
					<< std::endl << std::endl;
			return 0;
		}

		dataset_path = argv[1];
	}

	DepthClustering depth_clustering;

	if (!depth_clustering.initializeForDataset(dataset_path, dataset_file_type))
	{
		std::cout << "[ERROR]: Failed to initialize for dataset. Quit." << std::endl;
		return -1;
	}

	depth_clustering.processForDataset();
	depth_clustering.finish();

	return 0;
}
