/*
 * camera_projection.cpp
 *
 *  Created on: Sep 10, 2020
 *      Author: simonyu
 */

#include "post_processing/camera_projection.h"

CameraProjection::Parameter::Parameter() :
		intrinsic(), extrinsic(), width(0), height(0)
{
}

CameraProjection::CameraProjection(const Parameter& parameter)
{
	parameter_ = parameter;

	frame_ = std::make_shared<Frame>();
}

std::shared_ptr<CameraProjection::Frame>
CameraProjection::getFrame() const
{
	return frame_;
}

void
CameraProjection::setBoundingBox(std::shared_ptr<BoundingBox> bounding_box)
{
	bounding_box_ = bounding_box;
}

void
CameraProjection::clearFrame()
{
	frame_->clear();
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

void
CameraProjection::projectBoundingBoxFrameCube()
{
	if (!bounding_box_)
	{
		std::cout << "[ERROR]: Bounding box missing." << std::endl;
		return;
	}

	if (!bounding_box_->getFrameCube())
	{
		std::cout << "[ERROR]: Cube frame missing." << std::endl;
		return;
	}

	auto bounding_box_frame_cube = *(bounding_box_->getFrameCube());

	Eigen::Matrix<double, 3, 4> camera_to_image;
	Eigen::Matrix<double, 4, 4> world_to_camera_axes;
	Eigen::Matrix<double, 4, 4> world_to_camera;

	camera_to_image(0, 0) = parameter_.intrinsic[0];
	camera_to_image(0, 2) = parameter_.intrinsic[1];
	camera_to_image(1, 1) = parameter_.intrinsic[2];
	camera_to_image(1, 2) = parameter_.intrinsic[3];
	camera_to_image(2, 2) = 1;

	for (size_t i = 0; i < parameter_.extrinsic.size(); i ++)
	{
		world_to_camera(i) = parameter_.extrinsic[i];
	}

	world_to_camera_axes(0, 1) = -1;
	world_to_camera_axes(1, 2) = -1;
	world_to_camera_axes(2, 0) = 1;
	world_to_camera_axes(3, 3) = 1;

	world_to_camera = world_to_camera_axes * world_to_camera.inverse();

	world_to_camera(0, 3) = 0;
	world_to_camera(1, 3) = 0;
	world_to_camera(2, 3) = 0;
}

void
CameraProjection::projectBoundingBoxFramePolygon()
{
	std::cout << "[ERROR]: Not implemented." << std::endl;
}
