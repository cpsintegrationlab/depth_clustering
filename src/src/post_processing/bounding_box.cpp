/*
 * bounding_box.cpp
 *
 *  Created on: Sep 9, 2020
 *      Author: simonyu
 */

#include <boost/property_tree/json_parser.hpp>
#include <iostream>

#include "post_processing/bounding_box.h"

namespace depth_clustering
{

void
BoundingBox::OnNewObjectReceived(const NamedCluster& named_cluster, int)
{
	Timer timer;
	const auto &clouds = named_cluster.second;

	for (const auto &kv : clouds)
	{
		const auto &cluster = kv.second;

		switch (type_)
		{
		case Type::Cube:
			CreateCubes(std::make_pair(named_cluster.first, cluster));
			break;
		case Type::Polygon:
			CreatePolygons(std::make_pair(named_cluster.first, cluster));
			break;
		}
	}

	fprintf(stderr, "[INFO]: Bounding boxes created: %lu us.\n", timer.measure(Timer::Units::Micro));
}

void
BoundingBox::writeLog()
{
	if (log_)
	{
		boost::property_tree::write_json(log_file_, log_file_tree_);

		std::cout << "[INFO]: Wrote to log file '" << log_file_path_ + log_file_name_ << "'."
				<< std::endl;

		log_file_.close();
	}
}

void
BoundingBox::CreateCubes(const NamedCloud& named_cloud)
{
	const auto &cloud = named_cloud.second;
	Eigen::Vector3f center = Eigen::Vector3f::Zero();
	Eigen::Vector3f extent = Eigen::Vector3f::Zero();
	Eigen::Vector3f max_point(std::numeric_limits<float>::lowest(),
			std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
	Eigen::Vector3f min_point(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
			std::numeric_limits<float>::max());
	for (const auto &point : cloud.points())
	{
		center = center + point.AsEigenVector();
		min_point << std::min(min_point.x(), point.x()), std::min(min_point.y(), point.y()), std::min(
				min_point.z(), point.z());
		max_point << std::max(max_point.x(), point.x()), std::max(max_point.y(), point.y()), std::max(
				max_point.z(), point.z());
	}
	center /= cloud.size();
	if (min_point.x() < max_point.x())
	{
		extent = max_point - min_point;
	}

	if (log_)
	{
		logObject(named_cloud.first, center, extent);
	}

	frame_cube_->push_back(std::make_pair(center, extent));
}

void
BoundingBox::CreatePolygons(const NamedCloud& named_cloud)
{
	const auto &cloud = named_cloud.second;
	float min_z
	{ std::numeric_limits<float>::max() };
	float max_z
	{ std::numeric_limits<float>::lowest() };
	std::vector<cv::Point2f> cv_points;
	cv_points.reserve(cloud.size());
	std::vector<int> hull_indices;
	for (const auto &point : cloud.points())
	{
		cv_points.emplace_back(cv::Point2f
		{ point.x(), point.y() });
		min_z = std::min(min_z, point.z());
		max_z = std::max(max_z, point.z());
	}
	cv::convexHull(cv_points, hull_indices);
	AlignedEigenVectors hull;
	hull.reserve(cloud.size());
	for (int index : hull_indices)
	{
		const auto &cv_point = cv_points[index];
		hull.emplace_back(cv_point.x, cv_point.y, min_z);
	}
	const float diff_z = max_z - min_z;
	if (diff_z < 0.3)
	{
		return;
	}

	if (log_)
	{
		logObject(named_cloud.first, hull, diff_z);
	}

	frame_polygon_->push_back(std::make_pair(hull, diff_z));
}

void
BoundingBox::logObject(const std::string& file_name, const Eigen::Vector3f& center,
		const Eigen::Vector3f& extent)
{
	if (!log_file_.is_open())
	{
		openLogFile(file_name);

		if (!log_file_.is_open())
		{
			std::cout << "[WARN]: Failed to open log file '" << log_file_path_ + log_file_name_
					<< "'." << std::endl;
			return;
		}

		std::cout << "[INFO]: Opened log file '" << log_file_path_ + log_file_name_ << "'."
				<< std::endl;
	}

	boost::property_tree::ptree cloud_object_array_value;
	boost::property_tree::ptree cloud_object_array;
	std::string cloud_file_name = file_name;

	cloud_object_array_value.put_value(center.x());
	cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

	cloud_object_array_value.put_value(center.y());
	cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

	cloud_object_array_value.put_value(center.z());
	cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

	cloud_object_array_value.put_value(extent.x());
	cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

	cloud_object_array_value.put_value(extent.y());
	cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

	cloud_object_array_value.put_value(extent.z());
	cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

	cloud_file_name.erase(cloud_file_name.find(log_file_path_), log_file_path_.length());
	auto cloud_file_array_optional = log_file_tree_.get_child_optional(
			boost::property_tree::ptree::path_type(cloud_file_name, '/'));

	if (!cloud_file_array_optional)
	{
		log_file_tree_.add_child(boost::property_tree::ptree::path_type(cloud_file_name, '/'),
				boost::property_tree::ptree());
		auto &cloud_file_array = log_file_tree_.get_child(
				boost::property_tree::ptree::path_type(cloud_file_name, '/'));

		cloud_file_array.push_back(std::make_pair("", cloud_object_array));
	}
	else
	{
		auto &cloud_file_array = *cloud_file_array_optional;

		cloud_file_array.push_back(std::make_pair("", cloud_object_array));
	}
}

void
BoundingBox::logObject(const std::string& file_name, const AlignedEigenVectors& hull,
		const float& diff_z)
{
	if (!log_file_.is_open())
	{
		openLogFile(file_name);

		if (!log_file_.is_open())
		{
			std::cout << "[WARN]: Failed to open log file '" << log_file_path_ + log_file_name_
					<< "'." << std::endl;
			return;
		}

		std::cout << "[INFO]: Opened log file '" << log_file_path_ + log_file_name_ << "'."
				<< std::endl;
	}

	boost::property_tree::ptree cloud_object_array_value;
	boost::property_tree::ptree cloud_object_array;
	boost::property_tree::ptree cloud_hull_vector_array_value;
	boost::property_tree::ptree cloud_hull_vector_array;
	boost::property_tree::ptree cloud_hull_array;
	std::string cloud_file_name = file_name;

	for (const auto &hull_vector : hull)
	{
		cloud_hull_vector_array_value.put_value(hull_vector.x());
		cloud_hull_vector_array.push_back(std::make_pair("", cloud_hull_vector_array_value));

		cloud_hull_vector_array_value.put_value(hull_vector.y());
		cloud_hull_vector_array.push_back(std::make_pair("", cloud_hull_vector_array_value));

		cloud_hull_vector_array_value.put_value(hull_vector.z());
		cloud_hull_vector_array.push_back(std::make_pair("", cloud_hull_vector_array_value));

		cloud_hull_array.push_back(std::make_pair("", cloud_hull_vector_array));

		cloud_hull_vector_array.clear();
	}

	cloud_object_array.push_back(std::make_pair("", cloud_hull_array));

	cloud_object_array_value.put_value(diff_z);
	cloud_object_array.push_back(std::make_pair("", cloud_object_array_value));

	cloud_file_name.erase(cloud_file_name.find(log_file_path_), log_file_path_.length());
	auto cloud_file_array_optional = log_file_tree_.get_child_optional(
			boost::property_tree::ptree::path_type(cloud_file_name, '/'));

	if (!cloud_file_array_optional)
	{
		log_file_tree_.add_child(boost::property_tree::ptree::path_type(cloud_file_name, '/'),
				boost::property_tree::ptree());
		auto &cloud_file_array = log_file_tree_.get_child(
				boost::property_tree::ptree::path_type(cloud_file_name, '/'));

		cloud_file_array.push_back(std::make_pair("", cloud_object_array));
	}
	else
	{
		auto &cloud_file_array = *cloud_file_array_optional;

		cloud_file_array.push_back(std::make_pair("", cloud_object_array));
	}
}

void
BoundingBox::openLogFile(const std::string& file_name)
{
	char log_file_path_delim = '/';
	std::string log_file_path = "";
	std::string log_file_path_portion = "";
	std::vector<std::string> log_file_path_portions;
	std::istringstream ss(file_name);

	while (log_file_path_portion != std::string(1, log_file_path_delim))
	{
		log_file_path_portion = std::string(1, log_file_path_delim);
		getline(ss, log_file_path_portion, log_file_path_delim);
		log_file_path_portions.push_back(log_file_path_portion);
	}

	log_file_path_portions.pop_back();
	log_file_path_portions.pop_back();

	for (const auto &portion : log_file_path_portions)
	{
		log_file_path += portion;
		log_file_path += std::string(1, log_file_path_delim);
	}

	log_file_path_ = log_file_path;

	log_file_.close();
	log_file_.open(log_file_path_ + log_file_name_, std::fstream::out | std::fstream::trunc);
}

}  // namespace depth_clustering
