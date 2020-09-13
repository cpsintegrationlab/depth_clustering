/*
 * bounding_box.cpp
 *
 *  Created on: Sep 9, 2020
 *      Author: simonyu
 */

#include "post_processing/bounding_box.h"
#include "post_processing/camera_projection.h"

namespace depth_clustering
{

BoundingBox::BoundingBox() :
		BoundingBox(Type::Cube)
{
}

BoundingBox::BoundingBox(const Type& type) :
		type_(type)
{
	frame_cube_ = std::make_shared<Frame<Cube>>();
	frame_polygon_ = std::make_shared<Frame<Polygon>>();
	frame_flat_ = std::make_shared<Frame<Flat>>();
}

BoundingBox::BoundingBox(const Type& type,
		const CameraProjectionParameter& camera_projection_parameter) :
		BoundingBox(type)
{
	camera_projection_ = std::make_shared<CameraProjection>(camera_projection_parameter);
	camera_projection_->setFrames(frame_cube_, frame_polygon_, frame_flat_);
}

std::shared_ptr<BoundingBox::Frame<BoundingBox::Cube>>
BoundingBox::getFrameCube() const
{
	return frame_cube_;
}

std::shared_ptr<BoundingBox::Frame<BoundingBox::Polygon>>
BoundingBox::getFramePolygon() const
{
	return frame_polygon_;
}

std::shared_ptr<BoundingBox::Frame<BoundingBox::Flat>>
BoundingBox::getFrameFlat() const
{
	return frame_flat_;
}

void
BoundingBox::setFrameCube(std::shared_ptr<Frame<Cube>> frame_cube)
{
	frame_cube_ = frame_cube;
	camera_projection_->setFrames(frame_cube_, frame_polygon_, frame_flat_);
}

void
BoundingBox::setFramePolygon(std::shared_ptr<Frame<Polygon>> frame_polygon)
{
	frame_polygon_ = frame_polygon;
	camera_projection_->setFrames(frame_cube_, frame_polygon_, frame_flat_);
}

void
BoundingBox::setFrameFlat(std::shared_ptr<Frame<Flat>> frame_flat)
{
	frame_flat_ = frame_flat;
	camera_projection_->setFrames(frame_cube_, frame_polygon_, frame_flat_);
}

void
BoundingBox::clearFrames()
{
	frame_cube_->clear();
	frame_polygon_->clear();
	frame_flat_->clear();
}

void
BoundingBox::produceFrameFlat()
{
	if (!camera_projection_)
	{
		std::cout << "[WARN]: Camera projection missing." << std::endl;
		return;
	}

	camera_projection_->projectFromBoundingBoxFrame(type_);
}

void
BoundingBox::OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds, int)
{
	Timer timer;

	for (const auto &kv : clouds)
	{
		const auto &cluster = kv.second;

		switch (type_)
		{
		case Type::Cube:
		{
			CreateCubes(cluster);
			break;
		}
		case Type::Polygon:
		{
			CreatePolygons(cluster);
			break;
		}
		default:
		{
			CreateCubes(cluster);
			break;
		}
		}
	}

	fprintf(stderr, "[INFO]: Bounding boxes created: %lu us.\n",
			timer.measure(Timer::Units::Micro));
}

void
BoundingBox::CreateCubes(const Cloud& cloud)
{
	if (!frame_cube_)
	{
		std::cout << "[ERROR]: Cube frame missing." << std::endl;
		return;
	}

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

	frame_cube_->push_back(std::make_tuple(center, extent, 0));
}

void
BoundingBox::CreatePolygons(const Cloud& cloud)
{
	if (!frame_polygon_)
	{
		std::cout << "[ERROR]: Polygon frame missing." << std::endl;
		return;
	}

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

	frame_polygon_->push_back(std::make_pair(hull, diff_z));
}

} // namespace depth_clustering
