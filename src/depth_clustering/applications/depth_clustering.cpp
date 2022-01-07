/*
 * depth_clustering.cpp
 *
 *  Created on: Apr 28, 2020
 *      Author: simonyu
 */

#include "api/api.h"

using depth_clustering::DepthClustering;

void
printUsage(int argc, char* argv[])
{
	std::cout << "Depth Clustering Command-Line Application 2.8.0" << std::endl;
	std::cout << std::endl << "Usage:\t" << argv[0] << " [dataset segment path]" << std::endl;
	std::cout << "\t" << argv[0] << " [dataset segment path] [global config file]" << std::endl;
	std::cout << "\t" << argv[0]
			<< " [dataset segment path] [global config file] [use second return]" << std::endl;
}

int
main(int argc, char* argv[])
{
	std::string dataset_path = "";
	std::string file_path_name_config_global = "";
	bool second_return = false;

	if (argc > 1)
	{
		if (std::string(argv[1]) == "-h")
		{
			printUsage(argc, argv);
			return 0;
		}

		dataset_path = argv[1];

		if (dataset_path != "" && dataset_path[dataset_path.size() - 1] != '/')
		{
			dataset_path += "/";
		}

		if (argc > 2)
		{
			file_path_name_config_global = argv[2];
		}

		if (argc > 3)
		{
			try
			{
				second_return = static_cast<bool>(std::stoi(argv[3]));
			} catch (const std::exception &e)
			{
				printUsage(argc, argv);
				return 0;
			}
		}
	}
	else
	{
		printUsage(argc, argv);
		return 0;
	}

	DepthClustering depth_clustering;

	if (!depth_clustering.initializeForDataset(dataset_path, file_path_name_config_global,
			second_return, false))
	{
		std::cout << "[ERROR]: Failed to initialize for dataset. Quit." << std::endl;
		return -1;
	}

	const auto &frame_paths_names_range =
			depth_clustering.getFolderReaderRange()->GetAllFilePaths();
	const auto &frame_paths_names_intensity =
			depth_clustering.getFolderReaderIntensity()->GetAllFilePaths();
	const auto &frame_paths_names_elongation =
			depth_clustering.getFolderReaderElongation()->GetAllFilePaths();

	for (int frame_number = 0; frame_number < static_cast<int>(frame_paths_names_range.size());
			frame_number++)
	{
		std::cout << std::endl;
		std::string frame_path_name_range = "";
		std::string frame_path_name_intensity = "";
		std::string frame_path_name_elongation = "";

		if (frame_paths_names_range.empty()
				|| frame_number >= static_cast<int>(frame_paths_names_range.size()))
		{
			std::cerr << "[WARN]: Range image missing in \"" << dataset_path << "\"." << std::endl;
			continue;
		}
		else
		{
			frame_path_name_range = frame_paths_names_range[frame_number];
		}

		if (!frame_paths_names_intensity.empty()
				&& frame_number < static_cast<int>(frame_paths_names_range.size()))
		{
			frame_path_name_intensity = frame_paths_names_intensity[frame_number];
		}

		if (!frame_paths_names_elongation.empty()
				&& frame_number < static_cast<int>(frame_paths_names_range.size()))
		{
			frame_path_name_elongation = frame_paths_names_elongation[frame_number];
		}

		depth_clustering.processOneFrameForDataset(frame_path_name_range, frame_path_name_intensity,
				frame_path_name_elongation);
	}

	depth_clustering.writeLogForDataset();

	return 0;
}
