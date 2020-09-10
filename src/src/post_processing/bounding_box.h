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
#include "utils/cloud.h"
#include "utils/timer.h"

namespace depth_clustering
{

class BoundingBox: public depth_clustering::AbstractClient<NamedCluster>
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

	enum class Type
	{
		Cube, Polygon
	};

	BoundingBox();

	BoundingBox(const Type& type);

	void
	setFrame(std::shared_ptr<Frame<Cube>> frame_cube, std::shared_ptr<Frame<Polygon>> frame_polygon);

	void
	OnNewObjectReceived(const NamedCluster& named_cluster, int id) override;

private:

	void
	CreateCubes(const NamedCloud& named_cloud);

	void
	CreatePolygons(const NamedCloud& named_cloud);

	Type type_ = Type::Cube;
	std::shared_ptr<Frame<Cube>> frame_cube_;
	std::shared_ptr<Frame<Polygon>> frame_polygon_;
};

} // namespace depth_clustering

#endif /* SRC_POST_PROCESSING_BOUNDING_BOX_H_ */
