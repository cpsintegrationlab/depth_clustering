// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#ifndef SRC_QT_DRAWABLES_DRAWABLE_CUBE_H_
#define SRC_QT_DRAWABLES_DRAWABLE_CUBE_H_

#include <Eigen/Core>

#include "visualization/drawables/drawable.h"

namespace depth_clustering
{
class DrawableCube: public Drawable
{
public:

	DrawableCube(const Eigen::Vector3f& center, const Eigen::Vector3f& scale,
			const Eigen::Vector3f& color, const float& rotation = 0) :
			_center
			{ center }, _scale
			{ scale }, _color
			{ color }, rotation_(rotation)
	{
	}

	~DrawableCube() override
	{
	}

	void
	Draw() const override;

	static DrawableCube::UniquePtr
	Create(const Eigen::Vector3f& center, const Eigen::Vector3f& scale,
			const Eigen::Vector3f& color = Eigen::Vector3f(1.0f, 0.5f, 0.2f),
			const float& rotation = 0)
	{
		return std::unique_ptr<DrawableCube>(new DrawableCube(center, scale, color, rotation));
	}

private:

	Eigen::Vector3f _center;
	Eigen::Vector3f _scale;
	Eigen::Vector3f _color;
	float rotation_;
};
}  // namespace depth_clustering

#endif  // SRC_QT_DRAWABLES_DRAWABLE_CUBE_H_
