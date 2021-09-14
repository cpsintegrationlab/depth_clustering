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
#include "utils/timer.h"
#include "utils/velodyne_utils.h"

using boost::property_tree::json_parser::read_json;
using depth_clustering::DiffFactory;
using depth_clustering::MatFromPNGRange;
using depth_clustering::MatFromTIFFElongation;
using depth_clustering::MatFromTIFFIntensity;
using depth_clustering::MatFromTIFFRange;
using depth_clustering::Radians;
using depth_clustering::RichPoint;
using depth_clustering::time_utils::Timer;

DepthClustering::DepthClustering() :
		DepthClustering(DepthClusteringParameter())
{
}

DepthClustering::DepthClustering(const DepthClusteringParameter& parameter) :
		parameter_(parameter), dataset_path_("./"), frame_counter_(0)
{
}

const DepthClusteringParameter&
DepthClustering::getParameter() const
{
	return parameter_;
}

const CameraProjectionParameter&
DepthClustering::getCameraProjectionParameter() const
{
	return parameter_projection_camera_;
}

std::shared_ptr<ProjectionParams>
DepthClustering::getLidarProjectionParameter() const
{
	return parameter_projection_lidar_;
}

const std::string&
DepthClustering::getDatasetPath() const
{
	return dataset_path_;
}

Cloud::ConstPtr
DepthClustering::getCloudRange() const
{
	if (image_range_.rows == 0 || image_range_.cols == 0)
	{
		return nullptr;
	}

	return cloud_range_;
}

Cloud::ConstPtr
DepthClustering::getCloudIntensity() const
{
	if (image_range_.rows == 0 || image_range_.cols == 0 || image_intensity_.rows == 0
			|| image_intensity_.cols == 0)
	{
		return nullptr;
	}

	return Cloud::FromImageIntensity(image_range_, image_intensity_, *parameter_projection_lidar_);
}

Cloud::ConstPtr
DepthClustering::getCloudElongation() const
{
	if (image_range_.rows == 0 || image_range_.cols == 0 || image_elongation_.rows == 0
			|| image_elongation_.cols == 0)
	{
		return nullptr;
	}

	return Cloud::FromImageElongation(image_range_, image_elongation_, *parameter_projection_lidar_);
}

Cloud::ConstPtr
DepthClustering::getCloudConfidence() const
{
	if (image_range_.rows == 0 || image_range_.cols == 0 || image_intensity_.rows == 0
			|| image_intensity_.cols == 0 || image_elongation_.rows == 0
			|| image_elongation_.cols == 0)
	{
		return nullptr;
	}

	return Cloud::FromImageConfidence(image_range_, image_intensity_, image_elongation_,
			*parameter_projection_lidar_);
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

std::shared_ptr<DepthGroundRemover>
DepthClustering::getDepthGroundRemover() const
{
	return depth_ground_remover_;
}

void
DepthClustering::setParameter(const DepthClusteringParameter& parameter)
{
	parameter_ = parameter;

	if (parameter_.use_camera_fov)
	{
		auto projection_parameter_raw = parameter_projection_lidar_->getProjectionParamsRaw();

		projection_parameter_raw->updateHorizontalAngles(
				parameter_projection_camera_.field_of_view_angle_start,
				parameter_projection_camera_.field_of_view_angle_end);

		parameter_projection_lidar_ = ProjectionParams::FromBeamInclinations(
				projection_parameter_raw->horizontal_steps_current, projection_parameter_raw->beams,
				projection_parameter_raw->horizontal_angle_start,
				projection_parameter_raw->horizontal_angle_end,
				projection_parameter_raw->beam_inclinations);

		parameter_projection_lidar_->setProjectionParamsRaw(projection_parameter_raw);
	}
	else
	{
		parameter_projection_lidar_ = parameter_factory_->getLidarProjectionParameter();
	}

	auto logger_parameter = parameter_factory_->getLoggerParameter();

	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*parameter_projection_lidar_,
			parameter_.angle_ground_removal, parameter_.size_smooth_window);
	bounding_box_ = std::make_shared<BoundingBox>(parameter_.bounding_box_type,
			parameter_projection_camera_);

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

bool
DepthClustering::initializeForApollo()
{
	parameter_projection_lidar_ = ProjectionParams::APOLLO();
	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*parameter_projection_lidar_,
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
	parameter_projection_camera_ = parameter_factory_->getCameraProjectionParameter();
	parameter_projection_lidar_ = parameter_factory_->getLidarProjectionParameter();

	if (!parameter_projection_lidar_)
	{
		return false;
	}

	if (parameter_.use_camera_fov)
	{
		auto projection_parameter_raw = parameter_projection_lidar_->getProjectionParamsRaw();

		projection_parameter_raw->updateHorizontalAngles(
				parameter_projection_camera_.field_of_view_angle_start,
				parameter_projection_camera_.field_of_view_angle_end);

		parameter_projection_lidar_ = ProjectionParams::FromBeamInclinations(
				projection_parameter_raw->horizontal_steps_current, projection_parameter_raw->beams,
				projection_parameter_raw->horizontal_angle_start,
				projection_parameter_raw->horizontal_angle_end,
				projection_parameter_raw->beam_inclinations);

		parameter_projection_lidar_->setProjectionParamsRaw(projection_parameter_raw);
	}

	auto logger_parameter = parameter_factory_->getLoggerParameter();

	// Remove extrinsic translation vector since bounding boxes are already in camera frame
	if (parameter_projection_camera_.extrinsic.size() >= 12)
	{
		parameter_projection_camera_.extrinsic[3] = 0;
		parameter_projection_camera_.extrinsic[7] = 0;
		parameter_projection_camera_.extrinsic[11] = 0;
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

	depth_ground_remover_ = std::make_shared<DepthGroundRemover>(*parameter_projection_lidar_,
			parameter_.angle_ground_removal, parameter_.size_smooth_window);
	bounding_box_ = std::make_shared<BoundingBox>(parameter_.bounding_box_type,
			parameter_projection_camera_);
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

void
DepthClustering::processOneRangeFrameForApollo(const std::string& frame_name,
		const std::vector<Eigen::Vector3f>& point_cloud)
{
	std::cout << "[INFO]: Processing \"" << frame_name << "\"." << std::endl;

	Timer timer;
	cloud_range_ = Cloud::Ptr(new Cloud);

	for (const auto &point_eigen : point_cloud)
	{
		RichPoint point_rich;

		point_rich.x() = point_eigen.x();
		point_rich.y() = point_eigen.y();
		point_rich.z() = point_eigen.z();

		cloud_range_->push_back(point_rich);
	}

	cloud_range_->InitProjection(*parameter_projection_lidar_);
	bounding_box_->clearFrames();

	std::cout << "[INFO]: Preprocessed: " << timer.measure() << " us." << std::endl;

	depth_ground_remover_->OnNewObjectReceived(*cloud_range_, 0);

	std::cout << "[INFO]: Clustered: " << timer.measure() << " us." << std::endl;

	logger_->logBoundingBoxFrame(frame_name, parameter_.bounding_box_type);

	std::cout << "[INFO]: Logged: " << timer.measure() << " us." << std::endl;
}

const std::string
DepthClustering::processOneRangeFrameForDataset(const std::string& frame_path_name)
{
	if (frame_path_name == "")
	{
		std::cout << "[WARN]: Invalid frame path and name." << std::endl;
		return "";
	}

	Timer timer;
	std::string frame_name = "";
	std::stringstream ss(frame_path_name);

	while (std::getline(ss, frame_name, '/'))
	{
	}

	if (frame_name == "")
	{
		std::cout << "[WARN]: Invalid frame name." << std::endl;
		return "";
	}

	std::cout << "[INFO]: Processing \"" << frame_name << "\"." << std::endl;

	if (parameter_.dataset_file_type == ".png")
	{
		image_range_ = MatFromPNGRange(frame_path_name, parameter_projection_lidar_);
	}
	else if (parameter_.dataset_file_type == ".tiff")
	{
		image_range_ = MatFromTIFFRange(frame_path_name, parameter_projection_lidar_);
	}
	else
	{
		std::cout << "[WARN]: Unknown dataset file type." << std::endl;
		return "";
	}

	cloud_range_ = Cloud::FromImage(image_range_, *parameter_projection_lidar_);
	bounding_box_->clearFrames();

	std::cout << "[INFO]: Preprocessed: " << timer.measure() << " us." << std::endl;

	depth_ground_remover_->OnNewObjectReceived(*cloud_range_, 0);

	std::cout << "[INFO]: Clustered: " << timer.measure() << " us." << std::endl;

	bounding_box_->produceFrameFlat();

	std::cout << "[INFO]: Bounding box created: " << timer.measure() << " us." << std::endl;

	logger_->logBoundingBoxFrame(frame_name, parameter_.bounding_box_type);
	logger_->logBoundingBoxFrameFlat(frame_name);

	std::cout << "[INFO]: Logged: " << timer.measure() << " us." << std::endl;

	return frame_name;
}

const std::string
DepthClustering::processOneIntensityFrameForDataset(const std::string& frame_path_name)
{
	if (frame_path_name == "")
	{
		std::cout << "[WARN]: Invalid frame path and name." << std::endl;
		return "";
	}

	std::string frame_name = "";
	std::stringstream ss(frame_path_name);

	while (std::getline(ss, frame_name, '/'))
	{
	}

	if (frame_name == "")
	{
		std::cout << "[WARN]: Invalid frame name." << std::endl;
		return "";
	}

	std::cout << "[INFO]: Processing \"" << frame_name << "\"." << std::endl;

	if (parameter_.dataset_file_type == ".png")
	{
		std::cerr << "[ERROR]: The processing of \".png\" type intensity images is not implemented."
				<< std::endl;
		return "";
	}
	else if (parameter_.dataset_file_type == ".tiff")
	{
		image_intensity_ = MatFromTIFFIntensity(frame_path_name, parameter_projection_lidar_);
	}
	else
	{
		std::cout << "[WARN]: Unknown dataset file type." << std::endl;
		return "";
	}

	return frame_name;
}

const std::string
DepthClustering::processOneElongationFrameForDataset(const std::string& frame_path_name)
{
	if (frame_path_name == "")
	{
		std::cout << "[WARN]: Invalid frame path and name." << std::endl;
		return "";
	}

	std::string frame_name = "";
	std::stringstream ss(frame_path_name);

	while (std::getline(ss, frame_name, '/'))
	{
	}

	if (frame_name == "")
	{
		std::cout << "[WARN]: Invalid frame name." << std::endl;
		return "";
	}

	std::cout << "[INFO]: Processing \"" << frame_name << "\"." << std::endl;

	if (parameter_.dataset_file_type == ".png")
	{
		std::cerr
				<< "[ERROR]: The processing of \".png\" type elongation images is not implemented."
				<< std::endl;
		return "";
	}
	else if (parameter_.dataset_file_type == ".tiff")
	{
		image_elongation_ = MatFromTIFFElongation(frame_path_name, parameter_projection_lidar_);
	}
	else
	{
		std::cout << "[WARN]: Unknown dataset file type." << std::endl;
		return "";
	}

	return frame_name;
}

void
DepthClustering::processAllGroundTruthsForDataset()
{
	boost::property_tree::ptree ground_truth_tree;
	LoggerParameter parameter_logger;

	parameter_logger.log_path = dataset_path_;
	parameter_logger.log_file_name_flat = parameter_.ground_truth_flat_file_name;
	parameter_logger.log = true;

	std::shared_ptr<BoundingBox> bounding_box = std::make_shared<BoundingBox>(
			BoundingBox::Type::Cube, parameter_projection_camera_);
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

	std::cout << std::endl;
	logger->writeBoundingBoxLog(BoundingBox::Type::Flat);
}

void
DepthClustering::writeLogForApollo()
{
	std::cout << std::endl;
	logger_->writeBoundingBoxLog(parameter_.bounding_box_type);
}

void
DepthClustering::writeLogForDataset()
{
	std::cout << std::endl;
	logger_->writeBoundingBoxLog(parameter_.bounding_box_type);
	logger_->writeBoundingBoxLog(BoundingBox::Type::Flat);
}
