/*
 * camera_projection.cpp
 *
 *  Created on: Sep 10, 2020
 *      Author: simonyu
 */

#include "post_processing/camera_projection.h"

CameraProjection::Parameter::Parameter() :
		intrinsic(), extrinsic(), width(0), height(0), correct_distortions(false)
{
}

CameraProjection::CameraProjection(const Parameter& parameter) :
		parameter_(parameter)
{
	frame_flat_ = std::make_shared<BoundingBox::Frame<BoundingBox::Flat>>();
}

std::shared_ptr<BoundingBox::Frame<BoundingBox::Flat>>
CameraProjection::getFrameFlat() const
{
	return frame_flat_;
}

void
CameraProjection::setBoundingBox(std::shared_ptr<BoundingBox> bounding_box)
{
	bounding_box_ = bounding_box;
}

void
CameraProjection::clearFrame()
{
	frame_flat_->clear();
}

void
CameraProjection::projectBoundingBoxFrame(const BoundingBox::Type& bounding_box_type)
{
	switch (bounding_box_type)
	{
	case BoundingBox::Type::Cube:
	{
		projectBoundingBoxFrameCube();
		break;
	}
	case BoundingBox::Type::Polygon:
	{
		projectBoundingBoxFramePolygon();
		break;
	}
	default:
	{
		projectBoundingBoxFrameCube();
		break;
	}
	}
}

std::vector<Eigen::Vector3f>
CameraProjection::getBoundingBoxCornersCube(const BoundingBox::Cube& bounding_box)
{
	std::vector<Eigen::Vector3f> bounding_box_corners;
	auto bounding_box_center = bounding_box.first;
	auto bounding_box_extent = bounding_box.second;

	bounding_box_corners.push_back(
			Eigen::Vector3f(bounding_box_center.x() - bounding_box_extent.x() / 2,
					bounding_box_center.y() - bounding_box_extent.y() / 2,
					bounding_box_center.z() - bounding_box_extent.z() / 2));

	bounding_box_corners.push_back(
			Eigen::Vector3f(bounding_box_center.x() - bounding_box_extent.x() / 2,
					bounding_box_center.y() - bounding_box_extent.y() / 2,
					bounding_box_center.z() + bounding_box_extent.z() / 2));

	bounding_box_corners.push_back(
			Eigen::Vector3f(bounding_box_center.x() - bounding_box_extent.x() / 2,
					bounding_box_center.y() + bounding_box_extent.y() / 2,
					bounding_box_center.z() - bounding_box_extent.z() / 2));

	bounding_box_corners.push_back(
			Eigen::Vector3f(bounding_box_center.x() - bounding_box_extent.x() / 2,
					bounding_box_center.y() + bounding_box_extent.y() / 2,
					bounding_box_center.z() + bounding_box_extent.z() / 2));

	bounding_box_corners.push_back(
			Eigen::Vector3f(bounding_box_center.x() + bounding_box_extent.x() / 2,
					bounding_box_center.y() - bounding_box_extent.y() / 2,
					bounding_box_center.z() - bounding_box_extent.z() / 2));

	bounding_box_corners.push_back(
			Eigen::Vector3f(bounding_box_center.x() + bounding_box_extent.x() / 2,
					bounding_box_center.y() - bounding_box_extent.y() / 2,
					bounding_box_center.z() + bounding_box_extent.z() / 2));

	bounding_box_corners.push_back(
			Eigen::Vector3f(bounding_box_center.x() + bounding_box_extent.x() / 2,
					bounding_box_center.y() + bounding_box_extent.y() / 2,
					bounding_box_center.z() - bounding_box_extent.z() / 2));

	bounding_box_corners.push_back(
			Eigen::Vector3f(bounding_box_center.x() + bounding_box_extent.x() / 2,
					bounding_box_center.y() + bounding_box_extent.y() / 2,
					bounding_box_center.z() + bounding_box_extent.z() / 2));

	return bounding_box_corners;
}

std::vector<Eigen::Vector3f>
CameraProjection::getBoundingBoxCornersPolygon(const BoundingBox::Polygon& bounding_box)
{
	std::cout << "[ERROR]: Not implemented." << std::endl;
	return std::vector<Eigen::Vector3f>();
}

float
CameraProjection::getBoundingBoxDepth(const std::vector<Eigen::Vector3f>& bounding_box_corners)
{
	float depth = bounding_box_corners[0].norm();

	for (const auto &bounding_box_corner : bounding_box_corners)
	{
		float depth_current = bounding_box_corner.norm();

		if (depth_current < depth)
		{
			depth = depth_current;
		}
	}

	return depth;
}

BoundingBox::Flat
CameraProjection::getBoundingBoxFlat(
		const std::vector<Eigen::Vector2d>& bounding_box_corners_projected)
{
	Eigen::Vector2d bounding_box_flat_corner_upper_left;
	Eigen::Vector2d bounding_box_flat_corner_lower_right;

	bounding_box_flat_corner_upper_left.x() = std::numeric_limits<float>::max();
	bounding_box_flat_corner_lower_right.x() = std::numeric_limits<float>::min();
	bounding_box_flat_corner_upper_left.y() = std::numeric_limits<float>::max();
	bounding_box_flat_corner_lower_right.y() = std::numeric_limits<float>::min();

	// Find flat bounding box boundary
	for (const auto &bounding_box_corner_projected : bounding_box_corners_projected)
	{
		if (bounding_box_corner_projected.x() < bounding_box_flat_corner_upper_left.x())
		{
			bounding_box_flat_corner_upper_left.x() = bounding_box_corner_projected.x();
		}

		if (bounding_box_corner_projected.x() > bounding_box_flat_corner_lower_right.x())
		{
			bounding_box_flat_corner_lower_right.x() = bounding_box_corner_projected.x();
		}

		if (bounding_box_corner_projected.y() < bounding_box_flat_corner_upper_left.y())
		{
			bounding_box_flat_corner_upper_left.y() = bounding_box_corner_projected.y();
		}

		if (bounding_box_corner_projected.y() > bounding_box_flat_corner_lower_right.y())
		{
			bounding_box_flat_corner_lower_right.y() = bounding_box_corner_projected.y();
		}
	}

	// Enforce image boundary
	bounding_box_flat_corner_upper_left.x() = std::min<float>(
			std::max<float>(0, bounding_box_flat_corner_upper_left.x()), parameter_.width);
	bounding_box_flat_corner_lower_right.x() = std::min<float>(
			std::max<float>(0, bounding_box_flat_corner_lower_right.x()), parameter_.width);
	bounding_box_flat_corner_upper_left.y() = std::min<float>(
			std::max<float>(0, bounding_box_flat_corner_upper_left.y()), parameter_.height);
	bounding_box_flat_corner_lower_right.y() = std::min<float>(
			std::max<float>(0, bounding_box_flat_corner_lower_right.y()), parameter_.height);

	return std::make_tuple(bounding_box_flat_corner_upper_left,
			bounding_box_flat_corner_lower_right, 0);
}

Eigen::Vector2f
CameraProjection::correctCameraDistortions(const Eigen::Vector2f& point)
{
	if (!parameter_.correct_distortions)
	{
		return point;
	}

	Eigen::Vector2f point_corrected;
	double k1 = parameter_.intrinsic[4];
	double k2 = parameter_.intrinsic[5];
	double p1 = parameter_.intrinsic[6];
	double p2 = parameter_.intrinsic[7];
	double k3 = parameter_.intrinsic[8];

	double r = std::sqrt(std::pow(point.x(), 2) + std::pow(point.y(), 2));
	double radial_correction_factor = 1 + k1 * std::pow(r, 2) + k2 * std::pow(r, 4)
			+ k3 * std::pow(r, 6);
	double tangential_correction_factor_x = 2 * p1 * point.x() * point.y()
			+ p2 * (std::pow(r, 2) + 2 * std::pow(point.x(), 2));
	double tangential_correction_factor_y = 2 * p2 * point.x() * point.y()
			+ p1 * (std::pow(r, 2) + 2 * std::pow(point.y(), 2));

	point_corrected.x() = point.x() * radial_correction_factor;
	point_corrected.y() = point.y() * radial_correction_factor;
	point_corrected.x() += tangential_correction_factor_x;
	point_corrected.y() += tangential_correction_factor_y;

	return point_corrected;
}

void
CameraProjection::projectBoundingBoxFrameCube()
{
	if (!bounding_box_)
	{
		std::cout << "[ERROR]: Bounding box missing." << std::endl;
		return;
	}

	auto bounding_box_frame = bounding_box_->getFrameCube();

	if (!bounding_box_frame)
	{
		std::cout << "[ERROR]: Bounding box frame missing." << std::endl;
		return;
	}

	Eigen::Matrix4f world_to_camera; // Camera extrinsic matrix, [R|t]
	Eigen::Matrix4f world_to_camera_axes; // Camera extrinsic matrix axes transformation matrix

	// Construct camera extrinsic matrix
	for (size_t i = 0; i < parameter_.extrinsic.size(); i++)
	{
		world_to_camera(i / 4, i % 4) = parameter_.extrinsic[i];
	}

	// Construct camera extrinsic matrix axes transformation matrix
	world_to_camera_axes << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;

	// Inverse given camera-to-world extrinsic and transformation axes into camera frame
	world_to_camera = world_to_camera_axes * world_to_camera.inverse();

	// Remove extrinsic translation vector sicen bounding boxes are already in camera frame
	world_to_camera(0, 3) = 0;
	world_to_camera(1, 3) = 0;
	world_to_camera(2, 3) = 0;

	// Process all bounding boxes in the frame
	for (const auto &bounding_box : *bounding_box_frame)
	{
		std::vector<Eigen::Vector3f> bounding_box_corners_world = getBoundingBoxCornersCube(
				bounding_box);
		std::vector<Eigen::Vector2d> bounding_box_corners_projected;
		float bounding_box_depth = getBoundingBoxDepth(bounding_box_corners_world);
		bool bounding_box_invalid = false;

		for (const auto &bounding_box_corner_world : bounding_box_corners_world)
		{
			// Project from world frame into camera frame
			Eigen::Vector4f bounding_box_corner_camera = world_to_camera
					* Eigen::Vector4f(bounding_box_corner_world(0), bounding_box_corner_world(1),
							bounding_box_corner_world(2), 1);

			// Exclude bounding box with invalid depth
			if (bounding_box_corner_camera(2) < 0)
			{
				bounding_box_invalid = true;
				break;
			}

			// Project from 3D camera frame into 2D
			Eigen::Vector2f bounding_box_corner_projected(
					bounding_box_corner_camera(0) / bounding_box_corner_camera(2),
					bounding_box_corner_camera(1) / bounding_box_corner_camera(2));

			// Correct camera distortions
			bounding_box_corner_projected = correctCameraDistortions(bounding_box_corner_projected);

			// Project from 2D camera frame onto image frame by applying camera intrinsic matrix
			bounding_box_corner_projected.x() = std::round(
					bounding_box_corner_projected.x() * parameter_.intrinsic[0]
							+ parameter_.intrinsic[2]);
			bounding_box_corner_projected.y() = std::round(
					bounding_box_corner_projected.y() * parameter_.intrinsic[1]
							+ parameter_.intrinsic[3]);

			// Store projected corner
			bounding_box_corners_projected.push_back(
					Eigen::Vector2d(bounding_box_corner_projected(0),
							bounding_box_corner_projected(1)));
		}

		if (bounding_box_invalid || bounding_box_corners_projected.empty())
		{
			continue;
		}

		// Obtain 2D flat bounding box
		BoundingBox::Flat bounding_box_flat = getBoundingBoxFlat(bounding_box_corners_projected);
		std::get<2>(bounding_box_flat) = bounding_box_depth;

		// Store 2D flat bounding box
		frame_flat_->push_back(bounding_box_flat);
	}
}

void
CameraProjection::projectBoundingBoxFramePolygon()
{
	std::cout << "[ERROR]: Not implemented." << std::endl;
}
