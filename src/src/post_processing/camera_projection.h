/*
 * camera_projection.h
 *
 *  Created on: Sep 10, 2020
 *      Author: simonyu
 */

#ifndef SRC_POST_PROCESSING_CAMERA_PROJECTION_H_
#define SRC_POST_PROCESSING_CAMERA_PROJECTION_H_

#include "post_processing/bounding_box.h"

using depth_clustering::BoundingBox;

class CameraProjection
{
public:

	struct Parameter
	{
		std::vector<double> intrinsic;
		std::vector<double> extrinsic;
		int width;
		int height;
		bool correct_distortions;

		Parameter();
	};

	CameraProjection(const Parameter& parameter);

	std::shared_ptr<BoundingBox::Frame<BoundingBox::Flat>>
	getFrameFlat() const;

	void
	setBoundingBox(std::shared_ptr<BoundingBox> bounding_box);

	void
	clearFrame();

	void
	projectBoundingBoxFrame(const BoundingBox::Type& bounding_box_type);

private:

	std::vector<Eigen::Vector3f>
	getBoundingBoxCornersCube(const BoundingBox::Cube& bounding_box);

	std::vector<Eigen::Vector3f>
	getBoundingBoxCornersPolygon(const BoundingBox::Polygon& bounding_box);

	float
	getBoundingBoxDepth(const std::vector<Eigen::Vector3f>& bounding_box_corners);

	BoundingBox::Flat
	getBoundingBoxFlat(const std::vector<Eigen::Vector2d>& bounding_box_corners_projected);

	Eigen::Vector2f
	correctCameraDistortions(const Eigen::Vector2f& point);

	void
	projectBoundingBoxFrameCube();

	void
	projectBoundingBoxFramePolygon();

	Parameter parameter_;

	std::shared_ptr<BoundingBox::Frame<BoundingBox::Flat>> frame_flat_;
	std::shared_ptr<BoundingBox> bounding_box_;
};

#endif /* SRC_POST_PROCESSING_CAMERA_PROJECTION_H_ */
