/*
 * parameter_factory.h
 *
 *  Created on: Sep 10, 2020
 *      Author: simonyu
 */

#ifndef SRC_API_PARAMETER_FACTORY_H_
#define SRC_API_PARAMETER_FACTORY_H_

#include <boost/property_tree/ptree.hpp>

#include "api/api_parameter.h"
#include "post_processing/logger_parameter.h"
#include "projections/projection_params.h"

using depth_clustering::ProjectionParams;

class ParameterFactory
{
public:

	ParameterFactory(std::string& path);

	DepthClusteringParameter
	getDepthClusteringParameter();

	std::unique_ptr<ProjectionParams>
	getLidarProjectionParameter();

	CameraProjectionParameter
	getCameraProjectionParameter();

	LoggerParameter
	getLoggerParameter();

private:

	const std::string configuration_file_name_;

	boost::property_tree::ptree top_tree_;
	boost::optional<boost::property_tree::ptree> depth_clustering_tree_;
	boost::optional<boost::property_tree::ptree> lidar_projection_tree_;
	boost::optional<boost::property_tree::ptree> camera_projection_tree_;
	boost::optional<boost::property_tree::ptree> logger_tree_;
};

#endif /* SRC_API_PARAMETER_FACTORY_H_ */
