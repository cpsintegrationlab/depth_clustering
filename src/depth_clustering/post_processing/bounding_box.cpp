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
	id_ = 0;
	frame_cluster_ = std::make_shared<Frame<Cluster>>();
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

std::shared_ptr<BoundingBox::Frame<BoundingBox::Cluster>>
BoundingBox::getFrameCluster() const
{
	return frame_cluster_;
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
	frame_cluster_->clear();
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
BoundingBox::OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds, int id)
{
	for (const auto &kv : clouds)
	{
		BoundingBox::Cluster cluster = std::make_tuple(kv.second, calculateScore(kv.second),
				std::to_string(id_));

		frame_cluster_->push_back(cluster);

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

		id_++;
	}
}

float
BoundingBox::calculateScore(const Cloud& cloud)
{
	int point_counter = 0;
	int point_counter_invalid = 0;
	float point_score_total = 0;
	float cluster_score = -1;

	for (const auto &point : cloud.points())
	{
		if (point.score() < 0 || point.score() > 1)
		{
			point_counter_invalid++;
			point_counter++;
			continue;
		}

		point_score_total += point.score();
		point_counter++;
	}

	if (point_counter_invalid < point_counter)
	{
		cluster_score = point_score_total / point_counter;
	}

	return cluster_score;
}

void
BoundingBox::CreateCubes(const Cluster& cluster)
{
	if (!frame_cube_)
	{
		std::cout << "[ERROR]: Cube frame missing." << std::endl;
		return;
	}

	const Cloud &cloud = std::get<0>(cluster);
	const float &score = std::get<1>(cluster);
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

	frame_cube_->push_back(std::make_tuple(center, extent, 0, score, std::to_string(id_)));
}

void
BoundingBox::CreatePolygons(const Cluster& cluster)
{
	if (!frame_polygon_)
	{
		std::cout << "[ERROR]: Polygon frame missing." << std::endl;
		return;
	}

	const Cloud &cloud = std::get<0>(cluster);
	const float &score = std::get<1>(cluster);
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
	const float height = max_z - min_z;

	frame_polygon_->push_back(std::make_tuple(hull, height, score, std::to_string(id_)));
}

} // namespace depth_clustering
