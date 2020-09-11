/*
 * bounding_box.h
 *
 *  Created on: Sep 9, 2020
 *      Author: simonyu
 */

#ifndef SRC_POST_PROCESSING_BOUNDING_BOX_H_
#define SRC_POST_PROCESSING_BOUNDING_BOX_H_

#include <algorithm>
#include <fstream>
#include <limits>
#include <unordered_map>
#include <vector>

#include "clusterers/abstract_clusterer.h"
#include "communication/abstract_client.h"
#include "ground_removal/depth_ground_remover.h"
#include "post_processing/camera_projection_parameter.h"
#include "utils/cloud.h"
#include "utils/timer.h"

class CameraProjection;

namespace depth_clustering
{

class BoundingBox: public depth_clustering::AbstractClient<std::unordered_map<uint16_t, Cloud>>
{
public:

	using Cloud = depth_clustering::Cloud;
	using Timer = depth_clustering::time_utils::Timer;
	using AlignedEigenVectors =
	std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>;

	template<typename Type>
	using Frame = std::vector<Type>;
	using Cube = std::pair<Eigen::Vector3f, Eigen::Vector3f>;
	using Polygon = std::pair<AlignedEigenVectors, float>;
	using Flat = std::tuple<Eigen::Vector2d, Eigen::Vector2d, float>;

	enum class Type
	{
		Cube, Polygon, Flat
	};

	BoundingBox();

	BoundingBox(const Type& type);

	BoundingBox(const Type& type, const CameraProjectionParameter& camera_projection_parameter);

	std::shared_ptr<Frame<Cube>>
	getFrameCube() const;

	std::shared_ptr<Frame<Polygon>>
	getFramePolygon() const;

	std::shared_ptr<Frame<Flat>>
	getFrameFlat() const;

	void
	clearFrames();

	void
	produceFrameFlat();

	void
	OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds, int id) override;

private:

	void
	CreateCubes(const Cloud& cloud);

	void
	CreatePolygons(const Cloud& cloud);

	Type type_ = Type::Cube;

	std::shared_ptr<Frame<Cube>> frame_cube_;
	std::shared_ptr<Frame<Polygon>> frame_polygon_;
	std::shared_ptr<Frame<Flat>> frame_flat_;
	std::shared_ptr<CameraProjection> camera_projection_;
};

} // namespace depth_clustering

#endif /* SRC_POST_PROCESSING_BOUNDING_BOX_H_ */
