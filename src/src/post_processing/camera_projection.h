/*
 * camera_projection.h
 *
 *  Created on: Sep 10, 2020
 *      Author: simonyu
 */

#ifndef SRC_POST_PROCESSING_CAMERA_PROJECTION_H_
#define SRC_POST_PROCESSING_CAMERA_PROJECTION_H_

#include "post_processing/camera_projection_parameter.h"
#include "post_processing/bounding_box.h"

using depth_clustering::BoundingBox;

class CameraProjection
{
public:

	CameraProjection();

	CameraProjection(const CameraProjectionParameter& parameter);

	void
	setFrames(std::shared_ptr<BoundingBox::Frame<BoundingBox::Cube>> bounding_box_frame_cube,
			std::shared_ptr<BoundingBox::Frame<BoundingBox::Polygon>> bounding_box_frame_polygon,
			std::shared_ptr<BoundingBox::Frame<BoundingBox::Flat>> bounding_box_frame_flat);

	void
	projectFromBoundingBoxFrame(const BoundingBox::Type& bounding_box_type);

private:

	std::vector<Eigen::Vector3f>
	getBoundingBoxCornersCube(const BoundingBox::Cube& bounding_box);

	std::vector<Eigen::Vector3f>
	getBoundingBoxCornersPolygon(const BoundingBox::Polygon& bounding_box);

	float
	getBoundingBoxDepth(const std::vector<Eigen::Vector3f>& bounding_box_corners);

	std::shared_ptr<BoundingBox::Flat>
	getBoundingBoxFlat(const std::vector<Eigen::Vector2i>& bounding_box_corners_projected);

	void
	correctCameraDistortions(Eigen::Vector2f& point);

	bool
	filterBoundingBoxHeight(const Eigen::Vector3f& center);

	bool
	filterBoundingBoxTunnel(const Eigen::Vector3f& center, const float& depth);

	void
	projectFromBoundingBoxFrameCube();

	void
	projectFromBoundingBoxFramePolygon();

	CameraProjectionParameter parameter_;

	std::shared_ptr<BoundingBox::Frame<BoundingBox::Cube>> bounding_box_frame_cube_;
	std::shared_ptr<BoundingBox::Frame<BoundingBox::Polygon>> bounding_box_frame_polygon_;
	std::shared_ptr<BoundingBox::Frame<BoundingBox::Flat>> bounding_box_frame_flat_;
};

#endif /* SRC_POST_PROCESSING_CAMERA_PROJECTION_H_ */
