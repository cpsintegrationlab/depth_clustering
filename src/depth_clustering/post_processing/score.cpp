/*
 * score.cpp
 *
 *  Created on: Oct 13, 2021
 *      Author: simonyu
 */

#include <algorithm>

#include "post_processing/score.h"

namespace depth_clustering
{
Score::Score() :
		type_point_(TypePoint::Type_1), type_cluster_(TypeCluster::Type_1), type_frame_(
				TypeFrame::Type_1)
{
}

Score::Score(const TypePoint& type_point, const TypeCluster& type_cluster,
		const TypeFrame& type_frame) :
		type_point_(type_point), type_cluster_(type_cluster), type_frame_(type_frame)
{
}

void
Score::setTypePoint(const TypePoint& type_point)
{
	type_point_ = type_point;
}

void
Score::setTypeCluster(const TypeCluster& type_cluster)
{
	type_cluster_ = type_cluster;
}

void
Score::setTypeFrame(const TypeFrame& type_frame)
{
	type_frame_ = type_frame;
}

float
Score::calculatePointScore(const RichPoint& point)
{
	switch (type_point_)
	{
	case TypePoint::Type_1:
	{
		return calculatePointScoreType1(point);
	}
	case TypePoint::Type_2:
	{
		return calculatePointScoreType2(point);
	}
	default:
	{
		return calculatePointScoreType1(point);
	}
	}
}

float
Score::calculateClusterScore(const Cloud& cloud)
{
	switch (type_cluster_)
	{
	case TypeCluster::Type_1:
	{
		return calculateClusterScoreType1(cloud);
	}
	default:
	{
		return calculateClusterScoreType1(cloud);
	}
	}
}

float
Score::calculateFrameScore(const std::shared_ptr<BoundingBox::Frame<BoundingBox::Cluster>> frame)
{
	switch (type_frame_)
	{
	case TypeFrame::Type_1:
	{
		return calculateFrameScoreType1(frame);
	}
	default:
	{
		return calculateFrameScoreType1(frame);
	}
	}
}

float
Score::boundScore(const float& score)
{
	return std::min<float>(1, std::max<float>(0, score));
}

float
Score::calculatePointScoreType1(const RichPoint& point)
{
	/*
	 * Intensity over elongation
	 */

	float score = -1;

	if (point.intensity() < 0 || point.elongation() < 0)
	{
		return score;
	}

	float intensity_shifted = point.intensity() + 1; // 1 - 2
	float elongation_shifted = point.elongation() + 1; // 1 - 2

	score = intensity_shifted / elongation_shifted; // 0.5 - 2
	score = fabs((score - 0.5) / 1.5); // 0 - 1

	return boundScore(score);
}

float
Score::calculatePointScoreType2(const RichPoint& point)
{
	/*
	 * one over elongation
	 */

	float score = -1;

	if (point.elongation() < 0)
	{
		return score;
	}

	float elongation_shifted = point.elongation() + 1; // 1 - 2

	score = 1 / elongation_shifted; // 0.5 - 1
	score = fabs((score + 0.5) / 1.5); // 0 - 1

	return boundScore(score);
}

float
Score::calculateClusterScoreType1(const Cloud& cloud)
{
	/*
	 * Point score average
	 */

	int point_counter_invalid = 0;
	float point_score_total = 0;
	float cluster_score = -1;

	for (const auto &point : cloud.points())
	{
		if (point.score() < 0 || point.score() > 1)
		{
			point_counter_invalid++;
			continue;
		}

		point_score_total += point.score();
	}

	if (point_counter_invalid < static_cast<int>(cloud.points().size()))
	{
		cluster_score = point_score_total / static_cast<int>(cloud.points().size());
	}

	return boundScore(cluster_score);
}

float
Score::calculateFrameScoreType1(
		const std::shared_ptr<BoundingBox::Frame<BoundingBox::Cluster>> frame)
{
	/*
	 * Cluster score average
	 */

	int cluster_counter_invalid = 0;
	float cluster_score_total = 0;
	float frame_score = -1;

	for (const auto &cluster : *frame)
	{
		if (std::get<1>(cluster) < 0 || std::get<1>(cluster) > 1)
		{
			cluster_counter_invalid++;
			continue;
		}

		cluster_score_total += std::get<1>(cluster);
	}

	if (cluster_counter_invalid < static_cast<int>(frame->size()))
	{
		frame_score = cluster_score_total / static_cast<int>(frame->size());
	}

	return boundScore(frame_score);
}
} // namespace depth_clustering
