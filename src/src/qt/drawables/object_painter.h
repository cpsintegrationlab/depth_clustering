// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#ifndef SRC_QT_DRAWABLES_OBJECT_PAINTER_H_
#define SRC_QT_DRAWABLES_OBJECT_PAINTER_H_

#include "ground_removal/depth_ground_remover.h"
#include "clusterers/abstract_clusterer.h"
#include "communication/abstract_client.h"
#include "qt/drawables/drawable_cube.h"
#include "qt/drawables/drawable_polygon3d.h"
#include "qt/viewer/viewer.h"

#include <utils/cloud.h>
#include <utils/timer.h>

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <vector>

#include <opencv2/imgproc.hpp>

#include <boost/property_tree/ptree.hpp>

namespace depth_clustering
{

class ObjectPainter: public depth_clustering::AbstractClient<NamedCluster>
{
	using Cloud = depth_clustering::Cloud;
	using Timer = depth_clustering::time_utils::Timer;

public:

	using OutputBox = std::pair<Eigen::Vector3f, Eigen::Vector3f>;
	using OutputPolygon = std::pair<DrawablePolygon3d::AlignedEigenVectors, float>;
	using OutputBoxFrame = std::vector<OutputBox>;
	using OutputPolygonFrame = std::vector<OutputPolygon>;

	enum class OutlineType
	{
		kBox, kPolygon3d
	};

	explicit
	ObjectPainter(Viewer* viewer, OutlineType outline_type, OutputBoxFrame* output_box_frame,
			OutputPolygonFrame* output_polygon_frame, bool log) :
			viewer_(viewer), outline_type_(outline_type), output_box_frame_(output_box_frame), output_polygon_frame_(
					output_polygon_frame), log_(log)
	{
	}

	void
	OnNewObjectReceived(const NamedCluster& named_cluster, int id) override;

	void
	writeLog();

private:

	Drawable::UniquePtr
	CreateDrawableCube(const NamedCloud& named_cloud);

	Drawable::UniquePtr
	CreateDrawablePolygon3d(const NamedCloud& named_cloud);

	void
	logObject(const std::string& file_name, const Eigen::Vector3f& center,
			const Eigen::Vector3f& extent);

	void
	logObject(const std::string& file_name, const DrawablePolygon3d::AlignedEigenVectors& hull,
			const float& diff_z);

	void
	openLogFile(const std::string& file_name);

	Viewer *viewer_ = nullptr;
	OutlineType outline_type_ = OutlineType::kBox;
	OutputBoxFrame *output_box_frame_ = nullptr;
	OutputPolygonFrame *output_polygon_frame_ = nullptr;

	bool log_ = true;
	std::ofstream log_file_;
	std::string log_file_path_ = "";
	const std::string log_file_name_ = "object.json";
	boost::property_tree::ptree log_file_tree_;
};

}  // namespace depth_clustering

#endif  // SRC_QT_DRAWABLES_OBJECT_PAINTER_H_
