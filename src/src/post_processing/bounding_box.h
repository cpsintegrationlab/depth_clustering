/*
 * bounding_box.h
 *
 *  Created on: Sep 9, 2020
 *      Author: simonyu
 */

#ifndef SRC_POST_PROCESSING_BOUNDING_BOX_H_
#define SRC_POST_PROCESSING_BOUNDING_BOX_H_

#include <algorithm>
#include <boost/property_tree/ptree.hpp>
#include <fstream>
#include <limits>
#include <unordered_map>
#include <vector>

#include "clusterers/abstract_clusterer.h"
#include "communication/abstract_client.h"
#include "ground_removal/depth_ground_remover.h"
#include "utils/cloud.h"
#include "utils/timer.h"

namespace depth_clustering
{

class BoundingBox: public depth_clustering::AbstractClient<NamedCluster>
{
public:

	using Cloud = depth_clustering::Cloud;
	using Timer = depth_clustering::time_utils::Timer;
	using AlignedEigenVectors =
	std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>;
	using OutputBox = std::pair<Eigen::Vector3f, Eigen::Vector3f>;
	using OutputPolygon = std::pair<AlignedEigenVectors, float>;
	using OutputBoxFrame = std::vector<OutputBox>;
	using OutputPolygonFrame = std::vector<OutputPolygon>;

	enum class OutlineType
	{
		kBox, kPolygon3d
	};

	explicit
	BoundingBox(OutlineType outline_type, OutputBoxFrame* output_box_frame,
			OutputPolygonFrame* output_polygon_frame, bool log) :
	outline_type_(outline_type), output_box_frame_(output_box_frame), output_polygon_frame_(
			output_polygon_frame), log_(log)
	{
	}

	void
	OnNewObjectReceived(const NamedCluster& named_cluster, int id) override;

	void
	writeLog();

private:

	void
	CreateDrawableCube(const NamedCloud& named_cloud);

	void
	CreateDrawablePolygon3d(const NamedCloud& named_cloud);

	void
	logObject(const std::string& file_name, const Eigen::Vector3f& center,
			const Eigen::Vector3f& extent);

	void
	logObject(const std::string& file_name, const AlignedEigenVectors& hull, const float& diff_z);

	void
	openLogFile(const std::string& file_name);

	OutlineType outline_type_ = OutlineType::kBox;
	OutputBoxFrame *output_box_frame_ = nullptr;
	OutputPolygonFrame *output_polygon_frame_ = nullptr;

	bool log_ = true;
	std::ofstream log_file_;
	std::string log_file_path_ = "";
	const std::string log_file_name_ = "detection.json";
	boost::property_tree::ptree log_file_tree_;
};

}	// namespace depth_clustering

#endif	// SRC_POST_PROCESSING_BOUNDING_BOX_H_
