/*
 * api.cpp
 *
 *  Created on: Apr 27, 2020
 *      Author: simonyu
 */

#include <boost/property_tree/json_parser.hpp>

#include "api/api.h"
#include "api/parameter_factory.h"
#include "image_labelers/diff_helpers/diff_factory.h"
#include "utils/cloud.h"
#include "utils/radians.h"
#include "utils/rich_point.h"
#include "utils/velodyne_utils.h"

using boost::property_tree::json_parser::read_json;
using depth_clustering::DiffFactory;
using depth_clustering::MatFromPNGRange;
using depth_clustering::MatFromTIFFElongation;
using depth_clustering::MatFromTIFFIntensity;
using depth_clustering::MatFromTIFFRange;
using depth_clustering::Radians;
using depth_clustering::RichPoint;

DepthClustering::DepthClustering() :
		DepthClustering(DepthClusteringParameter())
{
}

DepthClustering::DepthClustering(const DepthClusteringParameter& parameter) :
		parameter_(parameter), dataset_path_("./"), frame_counter_(0)
{
}

bool
DepthClustering::initializeForApollo()
{
	projection_parameter_ = ProjectionParams::APOLLO();
	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*projection_parameter_,
			parameter_.angle_ground_removal, parameter_.size_smooth_window);
	bounding_box_ = std::make_shared<BoundingBox>(parameter_.bounding_box_type);
	logger_ = std::make_shared<Logger>();

	Radians clustering_threshold;

	switch (parameter_.difference_type)
	{
	case DiffFactory::DiffType::ANGLES:
	{
		clustering_threshold = parameter_.angle_clustering;
		break;
	}
	case DiffFactory::DiffType::ANGLES_PRECOMPUTED:
	{
		clustering_threshold = parameter_.angle_clustering;
		break;
	}
	case DiffFactory::DiffType::LINE_DIST:
	{
		clustering_threshold = Radians::FromRadians(parameter_.distance_clustering);
		break;
	}
	case DiffFactory::DiffType::LINE_DIST_PRECOMPUTED:
	{
		clustering_threshold = Radians::FromRadians(parameter_.distance_clustering);
		break;
	}
	case DiffFactory::DiffType::SIMPLE:
	{
		clustering_threshold = Radians::FromRadians(parameter_.distance_clustering);
		break;
	}
	default:
	{
		clustering_threshold = parameter_.angle_clustering;
		break;
	}
	}

	clusterer_ = std::make_shared<ImageBasedClusterer<LinearImageLabeler<>>>(clustering_threshold,
			parameter_.size_cluster_min, parameter_.size_cluster_max);

	clusterer_->SetDiffType(parameter_.difference_type);
	logger_->setBoundingBox(bounding_box_);

	depth_ground_remover_->AddClient(clusterer_.get());
	clusterer_->AddClient(bounding_box_.get());

	return true;
}

bool
DepthClustering::initializeForDataset(const std::string& dataset_path, const bool& second_return)
{
	dataset_path_ = dataset_path;

	if (dataset_path_[dataset_path_.size() - 1] != '/')
	{
		dataset_path_ += "/";
	}

	std::string dataset_path_lidar_return = "first_return";

	if (second_return)
	{
		dataset_path_lidar_return = "second_return";
	}

	parameter_factory_ = std::make_shared<ParameterFactory>(dataset_path_);
	parameter_ = parameter_factory_->getDepthClusteringParameter();
	projection_parameter_ = parameter_factory_->getLidarProjectionParameter();

	if (!projection_parameter_)
	{
		return false;
	}

	auto camera_projection_parameter = parameter_factory_->getCameraProjectionParameter();
	auto logger_parameter = parameter_factory_->getLoggerParameter();

	// Remove extrinsic translation vector since bounding boxes are already in camera frame
	if (camera_projection_parameter.extrinsic.size() >= 12)
	{
		camera_projection_parameter.extrinsic[3] = 0;
		camera_projection_parameter.extrinsic[7] = 0;
		camera_projection_parameter.extrinsic[11] = 0;
	}

	logger_parameter.log_path = dataset_path_;

	std::string dataset_path_range = dataset_path_ + "frames_lidar/" + dataset_path_lidar_return
			+ "/range/";
	std::string dataset_path_intensity = dataset_path_ + "frames_lidar/" + dataset_path_lidar_return
			+ "/intensity/";
	std::string dataset_path_elongation = dataset_path_ + "frames_lidar/"
			+ dataset_path_lidar_return + "/elongation/";

	folder_reader_range_ = std::make_shared<FolderReader>(dataset_path_range,
			parameter_.dataset_file_type, FolderReader::Order::SORTED);
	folder_reader_intensity_ = std::make_shared<FolderReader>(dataset_path_intensity,
			parameter_.dataset_file_type, FolderReader::Order::SORTED);
	folder_reader_elongation_ = std::make_shared<FolderReader>(dataset_path_elongation,
			parameter_.dataset_file_type, FolderReader::Order::SORTED);

	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*projection_parameter_,
			parameter_.angle_ground_removal, parameter_.size_smooth_window);
	bounding_box_ = std::make_shared<BoundingBox>(parameter_.bounding_box_type,
			camera_projection_parameter);
	logger_ = std::make_shared<Logger>(logger_parameter);

	Radians clustering_threshold;

	switch (parameter_.difference_type)
	{
	case DiffFactory::DiffType::ANGLES:
	{
		clustering_threshold = parameter_.angle_clustering;
		break;
	}
	case DiffFactory::DiffType::ANGLES_PRECOMPUTED:
	{
		clustering_threshold = parameter_.angle_clustering;
		break;
	}
	case DiffFactory::DiffType::LINE_DIST:
	{
		clustering_threshold = Radians::FromRadians(parameter_.distance_clustering);
		break;
	}
	case DiffFactory::DiffType::LINE_DIST_PRECOMPUTED:
	{
		clustering_threshold = Radians::FromRadians(parameter_.distance_clustering);
		break;
	}
	case DiffFactory::DiffType::SIMPLE:
	{
		clustering_threshold = Radians::FromRadians(parameter_.distance_clustering);
		break;
	}
	default:
	{
		clustering_threshold = parameter_.angle_clustering;
		break;
	}
	}

	clusterer_ = std::make_shared<ImageBasedClusterer<LinearImageLabeler<>>>(clustering_threshold,
			parameter_.size_cluster_min, parameter_.size_cluster_max);

	clusterer_->SetDiffType(parameter_.difference_type);
	logger_->setBoundingBox(bounding_box_);

	depth_ground_remover_->AddClient(clusterer_.get());
	clusterer_->AddClient(bounding_box_.get());

	return true;
}

const DepthClusteringParameter&
DepthClustering::getParameter() const
{
	return parameter_;
}

const std::string&
DepthClustering::getDatasetPath() const
{
	return dataset_path_;
}

const cv::Mat&
DepthClustering::getImageRange() const
{
	return image_range_;
}

const cv::Mat&
DepthClustering::getImageIntensity() const
{
	return image_intensity_;
}

const cv::Mat&
DepthClustering::getImageElongation() const
{
	return image_elongation_;
}

Cloud::ConstPtr
DepthClustering::getCloud() const
{
	return cloud_;
}

std::shared_ptr<BoundingBox>
DepthClustering::getBoundingBox() const
{
	return bounding_box_;
}

std::shared_ptr<BoundingBox::Frame<BoundingBox::Flat>>
DepthClustering::getBoundingBoxFrameFlat() const
{
	if (!bounding_box_)
	{
		return nullptr;
	}

	return bounding_box_->getFrameFlat();
}

std::shared_ptr<ImageBasedClusterer<LinearImageLabeler<>>>
DepthClustering::getClusterer() const
{
	return clusterer_;
}

std::shared_ptr<FolderReader>
DepthClustering::getFolderReaderRange() const
{
	return folder_reader_range_;
}

std::shared_ptr<FolderReader>
DepthClustering::getFolderReaderIntensity() const
{
	return folder_reader_intensity_;
}

std::shared_ptr<FolderReader>
DepthClustering::getFolderReaderElongation() const
{
	return folder_reader_elongation_;
}

std::shared_ptr<ProjectionParams>
DepthClustering::getProjectionParameter() const
{
	return projection_parameter_;
}

std::shared_ptr<DepthGroundRemover>
DepthClustering::getDepthGroundRemover() const
{
	return depth_ground_remover_;
}

void
DepthClustering::setParameter(const DepthClusteringParameter& parameter)
{
	parameter_ = parameter;

	auto camera_projection_parameter = parameter_factory_->getCameraProjectionParameter();
	auto logger_parameter = parameter_factory_->getLoggerParameter();

	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*projection_parameter_,
			parameter_.angle_ground_removal, parameter_.size_smooth_window);
	bounding_box_ = std::make_shared<BoundingBox>(parameter_.bounding_box_type,
			camera_projection_parameter);

	Radians clustering_threshold;

	switch (parameter_.difference_type)
	{
	case DiffFactory::DiffType::ANGLES:
	{
		clustering_threshold = parameter_.angle_clustering;
		break;
	}
	case DiffFactory::DiffType::ANGLES_PRECOMPUTED:
	{
		clustering_threshold = parameter_.angle_clustering;
		break;
	}
	case DiffFactory::DiffType::LINE_DIST:
	{
		clustering_threshold = Radians::FromRadians(parameter_.distance_clustering);
		break;
	}
	case DiffFactory::DiffType::LINE_DIST_PRECOMPUTED:
	{
		clustering_threshold = Radians::FromRadians(parameter_.distance_clustering);
		break;
	}
	case DiffFactory::DiffType::SIMPLE:
	{
		clustering_threshold = Radians::FromRadians(parameter_.distance_clustering);
		break;
	}
	default:
	{
		clustering_threshold = parameter_.angle_clustering;
		break;
	}
	}

	clusterer_ = std::make_shared<ImageBasedClusterer<LinearImageLabeler<>>>(clustering_threshold,
			parameter_.size_cluster_min, parameter_.size_cluster_max);

	clusterer_->SetDiffType(parameter_.difference_type);
	logger_->setBoundingBox(bounding_box_);

	depth_ground_remover_->AddClient(clusterer_.get());
	clusterer_->AddClient(bounding_box_.get());
}

void
DepthClustering::processOneRangeFrameForApollo(const std::string& frame_name,
		const std::vector<Eigen::Vector3f>& point_cloud)
{
	cloud_ = Cloud::Ptr(new Cloud);

	for (const auto &point_eigen : point_cloud)
	{
		RichPoint point_rich;

		point_rich.x() = point_eigen.x();
		point_rich.y() = point_eigen.y();
		point_rich.z() = point_eigen.z();

		cloud_->push_back(point_rich);
	}

	cloud_->InitProjection(*projection_parameter_);

	bounding_box_->clearFrames();
	depth_ground_remover_->OnNewObjectReceived(*cloud_, 0);

	logger_->logBoundingBoxFrame(frame_name, parameter_.bounding_box_type);
}

const std::string
DepthClustering::processOneRangeFrameForDataset(const std::string& frame_path_name)
{
	std::string frame_name = "";
	std::stringstream ss(frame_path_name);

	while (std::getline(ss, frame_name, '/'))
	{
	}

	if (parameter_.dataset_file_type == ".png")
	{
		image_range_ = MatFromPNGRange(frame_path_name);
	}
	else if (parameter_.dataset_file_type == ".tiff")
	{
		image_range_ = MatFromTIFFRange(frame_path_name);
	}
	else
	{
		std::cout << "[INFO]: Unknown data type. Skip." << std::endl;
		return "";
	}

	cloud_ = Cloud::FromImage(image_range_, *projection_parameter_);

	std::cout << "[INFO]: Processing \"" << frame_name << "\"." << std::endl;

	bounding_box_->clearFrames();
	depth_ground_remover_->OnNewObjectReceived(*cloud_, 0);
	bounding_box_->produceFrameFlat();

	return frame_name;
}

const std::string
DepthClustering::processOneIntensityFrameForDataset(const std::string& frame_path_name)
{
	std::string frame_name = "";
	std::stringstream ss(frame_path_name);

	while (std::getline(ss, frame_name, '/'))
	{
	}

	if (parameter_.dataset_file_type == ".png")
	{
		std::cerr << "[ERROR]: The processing of \".png\" type intensity images is not implemented."
				<< std::endl;
		return "";
	}
	else if (parameter_.dataset_file_type == ".tiff")
	{
		image_intensity_ = MatFromTIFFIntensity(frame_path_name);
	}
	else
	{
		std::cout << "[INFO]: Unknown data type. Skip." << std::endl;
		return "";
	}

	// TODO adapt below
//	cloud_ = Cloud::FromImage(image_range_, *projection_parameter_);

	std::cout << "[INFO]: Processing \"" << frame_name << "\"." << std::endl;

	return frame_name;
}

const std::string
DepthClustering::processOneElongationFrameForDataset(const std::string& frame_path_name)
{
	std::string frame_name = "";
	std::stringstream ss(frame_path_name);

	while (std::getline(ss, frame_name, '/'))
	{
	}

	if (parameter_.dataset_file_type == ".png")
	{
		std::cerr
				<< "[ERROR]: The processing of \".png\" type elongation images is not implemented."
				<< std::endl;
		return "";
	}
	else if (parameter_.dataset_file_type == ".tiff")
	{
		image_elongation_ = MatFromTIFFElongation(frame_path_name);
	}
	else
	{
		std::cout << "[WARN]: Unknown dataset file type." << std::endl;
		return "";
	}

	// TODO adapt below
	//	cloud_ = Cloud::FromImage(image_range_, *projection_parameter_);

	std::cout << "[INFO]: Processing \"" << frame_name << "\"." << std::endl;

	return frame_name;
}

const std::string
DepthClustering::processNextRangeFrameForDataset()
{
	std::string frame_name = "";
	const auto &frame_paths_names = folder_reader_range_->GetAllFilePaths();

	if (frame_counter_ >= static_cast<int>(frame_paths_names.size()))
	{
		return frame_name;
	}

	while (frame_name == "" && frame_counter_ < static_cast<int>(frame_paths_names.size()))
	{
		std::cout << std::endl;
		frame_name = processOneRangeFrameForDataset(frame_paths_names[frame_counter_++]);
	}

	return frame_name;
}

const std::string
DepthClustering::processLastRangeFrameForDataset()
{
	std::string frame_name = "";
	const auto &frame_paths_names = folder_reader_range_->GetAllFilePaths();

	if (frame_counter_ < 0)
	{
		return frame_name;
	}

	while (frame_name == "" && frame_counter_ >= 0)
	{
		std::cout << std::endl;
		frame_name = processOneRangeFrameForDataset(frame_paths_names[frame_counter_--]);
	}

	return frame_name;
}

void
DepthClustering::processAllRangeFramesForDataset()
{
	for (const auto &frame_path_name : folder_reader_range_->GetAllFilePaths())
	{
		std::cout << std::endl;
		const auto &frame_name = processOneRangeFrameForDataset(frame_path_name);

		if (frame_name != "")
		{
			logger_->logBoundingBoxFrame(frame_name, parameter_.bounding_box_type);
			logger_->logBoundingBoxFrameFlat(frame_name);
		}
	}
}

void
DepthClustering::processGroundTruthForDataset()
{
	boost::property_tree::ptree ground_truth_tree;
	CameraProjectionParameter parameter_camera_projection;
	LoggerParameter parameter_logger;

	parameter_camera_projection = parameter_factory_->getCameraProjectionParameter();
	parameter_camera_projection.threshold_truncation = 0.7;

	parameter_logger.log_path = dataset_path_;
	parameter_logger.log_file_name_flat = parameter_.ground_truth_flat_file_name;
	parameter_logger.log = true;

	std::shared_ptr<BoundingBox> bounding_box = std::make_shared<BoundingBox>(
			BoundingBox::Type::Cube, parameter_camera_projection);
	std::shared_ptr<Logger> logger = std::make_shared<Logger>(parameter_logger);
	std::shared_ptr<BoundingBox::Frame<BoundingBox::Cube>> bounding_box_frame_cube =
			std::make_shared<BoundingBox::Frame<BoundingBox::Cube>>();

	bounding_box->setFrameCube(bounding_box_frame_cube);
	logger->setBoundingBox(bounding_box);

	boost::property_tree::read_json(dataset_path_ + parameter_.ground_truth_file_name,
			ground_truth_tree);

	for (const auto &bounding_box_frame_cube_pair : ground_truth_tree)
	{
		const auto &bounding_box_frame_name_cube = bounding_box_frame_cube_pair.first;

		for (const auto &bounding_box_cube_array_pair : bounding_box_frame_cube_pair.second)
		{
			std::vector<std::string> bounding_box_cube_values;
			Eigen::Vector3f center;
			Eigen::Vector3f extent;
			float rotation;
			std::string id;

			for (const auto &bounding_box_cube_value_pair : bounding_box_cube_array_pair.second)
			{
				bounding_box_cube_values.push_back(
						bounding_box_cube_value_pair.second.get_value<std::string>());
			}

			center << std::stof(bounding_box_cube_values[0]), std::stof(
					bounding_box_cube_values[1]), std::stof(bounding_box_cube_values[2]);
			extent << std::stof(bounding_box_cube_values[3]), std::stof(
					bounding_box_cube_values[4]), std::stof(bounding_box_cube_values[5]);
			rotation = std::stof(bounding_box_cube_values[6]);
			id = bounding_box_cube_values[7];

			bounding_box_frame_cube->push_back(std::make_tuple(center, extent, rotation, id));
		}

		bounding_box->produceFrameFlat();
		logger->logBoundingBoxFrameFlat(bounding_box_frame_name_cube);
		bounding_box->clearFrames();
	}

	logger->writeBoundingBoxLog(BoundingBox::Type::Flat);
}

void
DepthClustering::logForApollo()
{
	logger_->writeBoundingBoxLog(parameter_.bounding_box_type);
}

void
DepthClustering::logForDataset()
{
	logger_->writeBoundingBoxLog(parameter_.bounding_box_type);
	logger_->writeBoundingBoxLog(BoundingBox::Type::Flat);
}
