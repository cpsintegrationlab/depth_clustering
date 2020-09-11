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

Logger::Parameter::Parameter() :
		log_path("./"), log_file_name_cude("depth_clustering_detection_cube.json"), log_file_name_polygon(
				"depth_clustering_detection_polygon.json"), log_file_name_flat(
				"depth_clustering_detection_flat.json"), log(true)
{

}

Logger::Logger() :
		Logger(Parameter())
{
}

Logger::Logger(const Parameter& parameter) :
		parameter_(parameter)
{
}

void
Logger::setBoundingBox(std::shared_ptr<BoundingBox> bounding_box)
{
	bounding_box_ = bounding_box;
}

void
Logger::setCameraProjection(std::shared_ptr<CameraProjection> camera_projection)
{
	camera_projection_ = camera_projection;
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
	case BoundingBox::Type::Flat:
	{
		logBoundingBoxFrameFlat(frame_name);
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
Logger::logBoundingBoxFrameCube(const std::string& frame_name)
{
	if (!parameter_.log)
	{
		return;
	}

	if (!bounding_box_)
	{
		std::cout << "[ERROR]: Bounding box missing." << std::endl;
		return;
	}

	auto bounding_box_frame_cube = bounding_box_->getFrameCube();

	if (!bounding_box_frame_cube)
	{
		std::cout << "[ERROR]: Cube bounding box frame missing." << std::endl;
		return;
	}

	for (const auto &cube : *bounding_box_frame_cube)
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

		auto cloud_file_array_optional = bounding_box_log_tree_cube_.get_child_optional(
				boost::property_tree::ptree::path_type(frame_name, '/'));

		if (!cloud_file_array_optional)
		{
			bounding_box_log_tree_cube_.add_child(
					boost::property_tree::ptree::path_type(frame_name, '/'),
					boost::property_tree::ptree());
			auto &cloud_file_array = bounding_box_log_tree_cube_.get_child(
					boost::property_tree::ptree::path_type(frame_name, '/'));

			cloud_file_array.push_back(std::make_pair("", cube_array));
		}
		else
		{
			auto &cloud_file_array = *cloud_file_array_optional;

			cloud_file_array.push_back(std::make_pair("", cube_array));
		}
	}
}

void
Logger::logBoundingBoxFramePolygon(const std::string& frame_name)
{
	if (!parameter_.log)
	{
		return;
	}

	if (!bounding_box_)
	{
		std::cout << "[ERROR]: Bounding box missing." << std::endl;
		return;
	}

	auto bounding_box_frame_polygon = bounding_box_->getFramePolygon();

	if (!bounding_box_frame_polygon)
	{
		std::cout << "[ERROR]: Polygon bounding box frame missing." << std::endl;
		return;
	}

	for (const auto &polygon : *bounding_box_frame_polygon)
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

		auto cloud_file_array_optional = bounding_box_log_tree_polygon_.get_child_optional(
				boost::property_tree::ptree::path_type(frame_name, '/'));

		if (!cloud_file_array_optional)
		{
			bounding_box_log_tree_polygon_.add_child(
					boost::property_tree::ptree::path_type(frame_name, '/'),
					boost::property_tree::ptree());
			auto &cloud_file_array = bounding_box_log_tree_polygon_.get_child(
					boost::property_tree::ptree::path_type(frame_name, '/'));

			cloud_file_array.push_back(std::make_pair("", cloud_object_array));
		}
		else
		{
			auto &cloud_file_array = *cloud_file_array_optional;

			cloud_file_array.push_back(std::make_pair("", cloud_object_array));
		}
	}
}

void
Logger::logBoundingBoxFrameFlat(const std::string& frame_name)
{
	if (!parameter_.log)
	{
		return;
	}

	if (!camera_projection_)
	{
		std::cout << "[ERROR]: Camera projection missing." << std::endl;
		return;
	}

	auto bounding_box_frame_flat = camera_projection_->getFrameFlat();

	if (!bounding_box_frame_flat)
	{
		std::cout << "[ERROR]: Flat bounding box frame missing." << std::endl;
		return;
	}
}

void
Logger::writeBoundingBoxLog(const BoundingBox::Type& bounding_box_type)
{
	if (!parameter_.log)
	{
		return;
	}

	if (parameter_.log_path[parameter_.log_path.size() - 1] != '/')
	{
		parameter_.log_path += "/";
	}

	std::ofstream bounding_box_log_file;

	bounding_box_log_file.close();

	switch (bounding_box_type)
	{
	case BoundingBox::Type::Cube:
	{
		bounding_box_log_file.open(parameter_.log_path + parameter_.log_file_name_cude,
				std::fstream::out | std::fstream::trunc);

		boost::property_tree::write_json(bounding_box_log_file, bounding_box_log_tree_cube_);

		std::cout << std::endl << "[INFO]: Wrote to log file \""
				<< parameter_.log_path + parameter_.log_file_name_cude << "\"." << std::endl;

		break;
	}
	case BoundingBox::Type::Polygon:
	{
		bounding_box_log_file.open(parameter_.log_path + parameter_.log_file_name_polygon,
				std::fstream::out | std::fstream::trunc);

		boost::property_tree::write_json(bounding_box_log_file, bounding_box_log_tree_polygon_);

		std::cout << std::endl << "[INFO]: Wrote to log file \""
				<< parameter_.log_path + parameter_.log_file_name_polygon << "\"." << std::endl;

		break;
	}
	case BoundingBox::Type::Flat:
	{
		bounding_box_log_file.open(parameter_.log_path + parameter_.log_file_name_flat,
				std::fstream::out | std::fstream::trunc);

		boost::property_tree::write_json(bounding_box_log_file, bounding_box_log_tree_flat_);

		std::cout << std::endl << "[INFO]: Wrote to log file \""
				<< parameter_.log_path + parameter_.log_file_name_flat << "\"." << std::endl;

		break;
	}
	default:
	{
		bounding_box_log_file.open(parameter_.log_path + parameter_.log_file_name_cude,
				std::fstream::out | std::fstream::trunc);

		boost::property_tree::write_json(bounding_box_log_file, bounding_box_log_tree_cube_);

		std::cout << std::endl << "[INFO]: Wrote to log file \""
				<< parameter_.log_path + parameter_.log_file_name_cude << "\"." << std::endl;

		break;
	}
	}

	bounding_box_log_file.close();
}

} // namespace depth_clustering
