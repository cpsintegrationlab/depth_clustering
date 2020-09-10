/*
 * logger.cpp
 *
 *  Created on: Sep 9, 2020
 *      Author: simonyu
 */

#include <boost/property_tree/json_parser.hpp>

#include "post_processing/logger.h"

namespace depth_clustering
{

Logger::Logger() :
		Logger(true)
{
}

Logger::Logger(const bool& log) :
		log_(log)
{
}

void
Logger::setBoundingBoxFrame(
		std::shared_ptr<BoundingBox::Frame<BoundingBox::Cube>> bounding_box_frame_cube,
		std::shared_ptr<BoundingBox::Frame<BoundingBox::Polygon>> bounding_box_frame_polygon)
{
	bounding_box_frame_cube_ = bounding_box_frame_cube;
	bounding_box_frame_polygon_ = bounding_box_frame_polygon;
}

void
Logger::logBoundingBoxFrame(const std::string& frame_name,
		const BoundingBox::Type& bounding_box_type)
{
	switch (bounding_box_type)
	{
	case BoundingBox::Type::Cube:
	{
		logBoundingBoxFrameCube(frame_name);
		break;
	}
	case BoundingBox::Type::Polygon:
	{
		logBoundingBoxFramePolygon(frame_name);
		break;
	}
	default:
	{
		logBoundingBoxFrameCube(frame_name);
		break;
	}
	}
}

void
Logger::writeBoundingBoxLog(std::string& path, const std::string& file_name)
{
	if (!log_)
	{
		return;
	}

	if (path[path.size() - 1] != '/')
	{
		path += "/";
	}

	bounding_box_log_file_.close();
	bounding_box_log_file_.open(path + file_name, std::fstream::out | std::fstream::trunc);

	boost::property_tree::write_json(bounding_box_log_file_, bounding_box_log_tree_);

	std::cout << "[INFO]: Wrote to log file '" << path + file_name << "'." << std::endl;

	bounding_box_log_file_.close();
}

void
Logger::logBoundingBoxFrameCube(const std::string& frame_name)
{
	if (!log_)
	{
		return;
	}

	if (!bounding_box_frame_cube_)
	{
		std::cout << "[ERROR]: Cube frame missing." << std::endl;
		return;
	}

	auto bounding_box_frame_cube = *bounding_box_frame_cube_;

	for (const auto &cube : bounding_box_frame_cube)
	{
		boost::property_tree::ptree cube_array_value;
		boost::property_tree::ptree cube_array;

		auto center = cube.first;
		auto extent = cube.second;

		cube_array_value.put_value(center.x());
		cube_array.push_back(std::make_pair("", cube_array_value));

		cube_array_value.put_value(center.y());
		cube_array.push_back(std::make_pair("", cube_array_value));

		cube_array_value.put_value(center.z());
		cube_array.push_back(std::make_pair("", cube_array_value));

		cube_array_value.put_value(extent.x());
		cube_array.push_back(std::make_pair("", cube_array_value));

		cube_array_value.put_value(extent.y());
		cube_array.push_back(std::make_pair("", cube_array_value));

		cube_array_value.put_value(extent.z());
		cube_array.push_back(std::make_pair("", cube_array_value));

		auto cloud_file_array_optional = bounding_box_log_tree_.get_child_optional(
				boost::property_tree::ptree::path_type(frame_name, '/'));

		if (!cloud_file_array_optional)
		{
			bounding_box_log_tree_.add_child(
					boost::property_tree::ptree::path_type(frame_name, '/'),
					boost::property_tree::ptree());
			auto &cloud_file_array = bounding_box_log_tree_.get_child(
					boost::property_tree::ptree::path_type(frame_name, '/'));

			cloud_file_array.push_back(std::make_pair("", cube_array));
		}
		else
		{
			auto &cloud_file_array = *cloud_file_array_optional;

			cloud_file_array.push_back(std::make_pair("", cube_array));
		}
	}

	bounding_box_frame_cube_->clear();
}

void
Logger::logBoundingBoxFramePolygon(const std::string& frame_name)
{
	if (!log_)
	{
		return;
	}

	if (!bounding_box_frame_polygon_)
	{
		std::cout << "[ERROR]: Polygon frame missing." << std::endl;
		return;
	}

	auto bounding_box_frame_polygon = *bounding_box_frame_polygon_;

	for (const auto &polygon : bounding_box_frame_polygon)
	{
		boost::property_tree::ptree cloud_object_array_value;
		boost::property_tree::ptree cloud_object_array;
		boost::property_tree::ptree cloud_hull_vector_array_value;
		boost::property_tree::ptree cloud_hull_vector_array;
		boost::property_tree::ptree cloud_hull_array;

		auto hull = polygon.first;
		auto diff_z = polygon.second;

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

		auto cloud_file_array_optional = bounding_box_log_tree_.get_child_optional(
				boost::property_tree::ptree::path_type(frame_name, '/'));

		if (!cloud_file_array_optional)
		{
			bounding_box_log_tree_.add_child(
					boost::property_tree::ptree::path_type(frame_name, '/'),
					boost::property_tree::ptree());
			auto &cloud_file_array = bounding_box_log_tree_.get_child(
					boost::property_tree::ptree::path_type(frame_name, '/'));

			cloud_file_array.push_back(std::make_pair("", cloud_object_array));
		}
		else
		{
			auto &cloud_file_array = *cloud_file_array_optional;

			cloud_file_array.push_back(std::make_pair("", cloud_object_array));
		}
	}

	bounding_box_frame_polygon_->clear();
}

} // namespace depth_clustering
