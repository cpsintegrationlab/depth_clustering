/*
 * depth_clustering.cpp
 *
 *  Created on: Apr 28, 2020
 *      Author: simonyu
 */

#include "api/api.h"

int
main(int argc, char* argv[])
{
	std::string dataset_path = "../../../data/segment-2506799708748258165_6455_000_6475_000/";

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

	depth_clustering.processAllFramesForDataset();
	depth_clustering.logForDataset();

	return 0;
}
