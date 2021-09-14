/*
 * ground_truth_projection.cpp
 *
 *  Created on: Sep 13, 2020
 *      Author: simonyu
 */

#include "api/api.h"

int
main(int argc, char* argv[])
{
	std::string dataset_path;
	std::string global_config_path;

	if (argc > 1)
	{
		if (std::string(argv[1]) == "-h")
		{
			std::cout << std::endl << "Usage:\t" << argv[0] << " [dataset path]" << std::endl;
			std::cout << "\t" << argv[0] << " [dataset path] [global config path]" << std::endl
					<< std::endl;
			return 0;
		}

		dataset_path = argv[1];

		if (dataset_path != "" && dataset_path[dataset_path.size() - 1] != '/')
		{
			dataset_path += "/";
		}

		if (argc > 2)
		{
			global_config_path = argv[2];

			if (global_config_path != ""
					&& global_config_path[global_config_path.size() - 1] != '/')
			{
				global_config_path += "/";
			}
		}
	}
	else
	{
		std::cout << std::endl << "Usage:\t" << argv[0] << " [dataset path]" << std::endl;
		std::cout << "\t" << argv[0] << " [dataset path] [global config path]" << std::endl
				<< std::endl;
		return 0;
	}

	DepthClustering depth_clustering;

	if (!depth_clustering.initializeForDataset(dataset_path, global_config_path))
	{
		std::cout << "[ERROR]: Failed to initialize for dataset. Quit." << std::endl;
		return -1;
	}

	depth_clustering.processAllGroundTruthsForDataset();

	return 0;
}
