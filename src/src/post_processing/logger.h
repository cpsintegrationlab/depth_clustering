/*
 * logger.h
 *
 *  Created on: Sep 9, 2020
 *      Author: simonyu
 */

#ifndef SRC_POST_PROCESSING_LOGGER_H_
#define SRC_POST_PROCESSING_LOGGER_H_

#include <boost/property_tree/ptree.hpp>

#include "post_processing/bounding_box.h"

namespace depth_clustering
{

class Logger
{
public:

	Logger();

	Logger(const bool& log);

	void
	setBoundingBox(std::shared_ptr<BoundingBox> bounding_box);

	void
	logBoundingBoxFrame(const std::string& frame_name, const BoundingBox::Type& bounding_box_type);

	void
	writeBoundingBoxLog(std::string& path, const std::string& file_name);

private:

	void
	logBoundingBoxFrameCube(const std::string& frame_name);

	void
	logBoundingBoxFramePolygon(const std::string& frame_name);

	bool log_ = true;

	std::shared_ptr<BoundingBox> bounding_box_;
	std::ofstream bounding_box_log_file_;
	boost::property_tree::ptree bounding_box_log_tree_;
};

} // namespace depth_clustering

#endif /* SRC_POST_PROCESSING_LOGGER_H_ */
