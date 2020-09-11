/*
 * parameter_factory.cpp
 *
 *  Created on: Sep 10, 2020
 *      Author: simonyu
 */

#include <boost/property_tree/json_parser.hpp>

#include "api/parameter_factory.h"

using boost::property_tree::json_parser::read_json;

ParameterFactory::ParameterFactory(std::string& path) :
		configuration_file_name_("depth_clustering_config.json")
{
	if (path[path.size() - 1] != '/')
	{
		path += "/";
	}

	boost::property_tree::read_json(path + configuration_file_name_, top_tree_);

	depth_clustering_tree_ = top_tree_.get_child_optional("depth_clustering");
	lidar_projection_tree_ = top_tree_.get_child_optional("lidar_projection");
	camera_projection_tree_ = top_tree_.get_child_optional("camera_projection");
}

DepthClustering::Parameter
ParameterFactory::getDepthClusteringParameter()
{
	if (!depth_clustering_tree_)
	{
		std::cout << "[ERROR]: Depth clustering configuration missing." << std::endl;
		return DepthClustering::Parameter();
	}

	DepthClustering::Parameter parameter;

	auto tree = *depth_clustering_tree_;

	auto angle_clustering_optional = tree.get_optional<float>("angle_clustering");
	auto angle_ground_removal_optional = tree.get_optional<float>("angle_ground_removal");
	auto size_cluster_min_optional = tree.get_optional<int>("size_cluster_min");
	auto size_cluster_max_optional = tree.get_optional<int>("size_cluster_max");
	auto size_smooth_window_optional = tree.get_optional<int>("size_smooth_window");
	auto bounding_box_type_optional = tree.get_optional<std::string>("bounding_box_type");
	auto dataset_file_type_optional = tree.get_optional<std::string>("dataset_file_type");
	auto log_file_name_optional = tree.get_optional<std::string>("log_file_name");
	auto log_optional = tree.get_optional<bool>("log");

	if (angle_clustering_optional)
	{
		parameter.angle_clustering = Radians
		{ Radians::IsRadians
		{ }, static_cast<float>(*angle_clustering_optional * M_PI / 180.0) };
	}

	if (angle_ground_removal_optional)
	{
		parameter.angle_ground_removal = Radians
		{ Radians::IsRadians
		{ }, static_cast<float>(*angle_ground_removal_optional * M_PI / 180.0) };
	}

	if (size_cluster_min_optional)
	{
		parameter.size_cluster_min = *size_cluster_min_optional;
	}

	if (size_cluster_max_optional)
	{
		parameter.size_cluster_max = *size_cluster_max_optional;
	}

	if (size_smooth_window_optional)
	{
		parameter.size_smooth_window = *size_smooth_window_optional;
	}

	if (bounding_box_type_optional)
	{
		std::string bounding_box_type_string = *bounding_box_type_optional;

		if (bounding_box_type_string == "cube")
		{
			parameter.bounding_box_type = BoundingBox::Type::Cube;
		}
		else if (bounding_box_type_string == "polygon")
		{
			parameter.bounding_box_type = BoundingBox::Type::Polygon;
		}
	}

	if (dataset_file_type_optional)
	{
		parameter.dataset_file_type = *dataset_file_type_optional;
	}

	if (log_file_name_optional)
	{
		parameter.log_file_name = *log_file_name_optional;
	}

	if (log_optional)
	{
		parameter.log = *log_optional;
	}

	return parameter;
}

std::unique_ptr<ProjectionParams>
ParameterFactory::getLidarProjectionParameter()
{
	if (!lidar_projection_tree_)
	{
		std::cout << "[ERROR]: Lidar projection configuration missing." << std::endl;
		return nullptr;
	}

	ProjectionParams parameter;
	int horizontal_steps;
	int beams;
	int horizontal_angle_start;
	int horizontal_angle_end;
	std::vector<double> beam_inclinations;

	auto tree = *lidar_projection_tree_;

	auto horizontal_steps_optional = tree.get_optional<int>("horizontal_steps");
	auto beams_optional = tree.get_optional<int>("beams");
	auto horizontal_angle_start_optional = tree.get_optional<int>("horizontal_angle_start");
	auto horizontal_angle_end_optional = tree.get_optional<int>("horizontal_angle_end");
	auto beam_inclinations_optional = tree.get_child_optional("beam_inclinations");

	if (horizontal_steps_optional)
	{
		horizontal_steps = *horizontal_steps_optional;
	}

	if (beams_optional)
	{
		beams = *beams_optional;
	}

	if (horizontal_angle_start_optional)
	{
		horizontal_angle_start = *horizontal_angle_start_optional;
	}

	if (horizontal_angle_end_optional)
	{
		horizontal_angle_end = *horizontal_angle_end_optional;
	}

	if (beam_inclinations_optional)
	{
		for (const auto &beam_inclinations_pair : *beam_inclinations_optional)
		{
			beam_inclinations.push_back(beam_inclinations_pair.second.get_value<double>());
		}
	}

	return ProjectionParams::FromBeamInclinations(horizontal_steps, beams, horizontal_angle_start,
			horizontal_angle_end, beam_inclinations);
}

CameraProjection::Parameter
ParameterFactory::getCameraProjectionParameter()
{
	if (!camera_projection_tree_)
	{
		std::cout << "[ERROR]: Camera projection configuration missing." << std::endl;
		return CameraProjection::Parameter();
	}

	CameraProjection::Parameter parameter;

	auto tree = *camera_projection_tree_;

	auto intrinsic_optional = tree.get_child_optional("intrinsic");
	auto extrinsic_optional = tree.get_child_optional("extrinsic");
	auto width_optional = tree.get_optional<int>("width");
	auto height_optional = tree.get_optional<int>("height");
	auto correct_distortions_optional = tree.get_optional<bool>("correct_distortions");

	if (intrinsic_optional)
	{
		for (const auto &intrinsic_pair : *intrinsic_optional)
		{
			parameter.intrinsic.push_back(intrinsic_pair.second.get_value<double>());
		}
	}

	if (extrinsic_optional)
	{
		for (const auto &extrinsic_pair : *extrinsic_optional)
		{
			parameter.extrinsic.push_back(extrinsic_pair.second.get_value<double>());
		}
	}

	if (width_optional)
	{
		parameter.width = *width_optional;
	}

	if (height_optional)
	{
		parameter.height = *height_optional;
	}

	if (correct_distortions_optional)
	{
		parameter.correct_distortions = *correct_distortions_optional;
	}

	return parameter;
}
