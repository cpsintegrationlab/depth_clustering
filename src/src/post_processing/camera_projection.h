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

	using Projection = std::tuple<Eigen::Vector2f, Eigen::Vector2f, float>;
	using Frame = std::vector<Projection>;

	struct Parameter
	{
		std::vector<double> camera_intrinsic;
		std::vector<double> camera_extrinsic;

		Parameter();
	};

	CameraProjection();

	CameraProjection(const Parameter& parameter);

	std::shared_ptr<Frame>
	getFrame() const;

	void
	setBoundingBox(std::shared_ptr<BoundingBox> bounding_box);

	void
	clearFrame();

	void
	projectBoundingBoxFrame(const BoundingBox::Type& bounding_box_type);

private:

	void
	projectBoundingBoxFrameCube();

	void
	projectBoundingBoxFramePolygon();



	Parameter parameter_;

	std::shared_ptr<Frame> frame_;
	std::shared_ptr<BoundingBox> bounding_box_;
};

#endif /* SRC_POST_PROCESSING_CAMERA_PROJECTION_H_ */
