/*
 * parameter_factory.h
 *
 *  Created on: Sep 10, 2020
 *      Author: simonyu
 */

#ifndef SRC_API_PARAMETER_FACTORY_H_
#define SRC_API_PARAMETER_FACTORY_H_

#include <boost/property_tree/ptree.hpp>

#include "api/depth_clustering.h"
#include "post_processing/camera_projection.h"

class ParameterFactory
{
public:

	ParameterFactory(std::string& path);

	DepthClustering::Parameter
	getDepthClusteringParameter();

	std::unique_ptr<ProjectionParams>
	getLidarProjectionParameter();

	CameraProjection::Parameter
	getCameraProjectionParameter();

private:

	const std::string configuration_file_name_;

	boost::property_tree::ptree top_tree_;
	boost::optional<boost::property_tree::ptree> depth_clustering_tree_;
	boost::optional<boost::property_tree::ptree> lidar_projection_tree_;
	boost::optional<boost::property_tree::ptree> camera_projection_tree_;
};

#endif /* SRC_API_PARAMETER_FACTORY_H_ */
