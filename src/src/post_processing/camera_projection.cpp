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
}

void
CameraProjection::projectBoundingBoxFramePolygon()
{
	std::cout << "[ERROR]: Not implemented." << std::endl;
}
