// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#include <algorithm>
#include <cmath>
#include <GL/glut.h>

#include "visualization/drawables/drawable_cloud.h"

void
DrawableCloud::Draw() const
{
	if (!_cloud_ptr)
	{
		throw std::runtime_error("DrawableCloud has no cloud to draw.");
	}

	glPushMatrix();
	glBegin(GL_POINTS);

	for (const auto &point : _cloud_ptr->points())
	{
		if (color_from_intensity_)
		{
			setRGBColorWithValue(point.intensity());
		}
		else if (color_from_elongation_)
		{
			setRGBColorWithValue(point.elongation());
		}
		else if (color_from_score_)
		{
			setRGBColorWithValue(point.score());
		}
		else if (color_from_value_)
		{
			setRGBColorWithValue(value_);
		}
		else
		{
			glColor3f(_color[0], _color[1], _color[2]);
		}

		auto real_point = _cloud_ptr->pose() * point.AsEigenVector();
		glVertex3f(real_point.x(), real_point.y(), real_point.z());
	}

	glEnd();
	glPopMatrix();
}

DrawableCloud::Ptr
DrawableCloud::FromCloud(const Cloud::ConstPtr& cloud, const Eigen::Vector3f& color)
{
	return std::make_shared<DrawableCloud>(DrawableCloud(cloud, color));
}

DrawableCloud::Ptr
DrawableCloud::FromCloudWithIntensity(const Cloud::ConstPtr& cloud)
{
	return std::make_shared<DrawableCloud>(
			DrawableCloud(cloud, Eigen::Vector3f::Ones(), -1, true, false, false, false));
}

DrawableCloud::Ptr
DrawableCloud::FromCloudWithElongation(const Cloud::ConstPtr& cloud)
{
	return std::make_shared<DrawableCloud>(
			DrawableCloud(cloud, Eigen::Vector3f::Ones(), -1, false, true, false, false));
}

DrawableCloud::Ptr
DrawableCloud::FromCloudWithScore(const Cloud::ConstPtr& cloud)
{
	return std::make_shared<DrawableCloud>(
			DrawableCloud(cloud, Eigen::Vector3f::Ones(), -1, false, false, true, false));
}

DrawableCloud::Ptr
DrawableCloud::FromCloudWithValue(const Cloud::ConstPtr& cloud, const float& value)
{
	return std::make_shared<DrawableCloud>(
			DrawableCloud(cloud, Eigen::Vector3f::Ones(), value, false, false, false, true));
}

void
DrawableCloud::setRGBColorWithValue(const float& value) const
{
	if (value < 0 || value > 1)
	{
		glColor3f(_color[0], _color[1], _color[2]);
		return;
	}

	int value_int_low = std::floor(value * 255);
	int value_int_high = std::min(255, value_int_low + 1);
	float value_fraction = value * 255 - value_int_low;

	// https://ai.googleblog.com/2019/08/turbo-improved-rainbow-colormap-for.html
	float red = colormap_turbo[value_int_low][0]
			+ (colormap_turbo[value_int_high][0] - colormap_turbo[value_int_low][0])
					* value_fraction;
	float green = colormap_turbo[value_int_low][1]
			+ (colormap_turbo[value_int_high][1] - colormap_turbo[value_int_low][1])
					* value_fraction;
	float blue = colormap_turbo[value_int_low][2]
			+ (colormap_turbo[value_int_high][2] - colormap_turbo[value_int_low][2])
					* value_fraction;

	glColor3f(red, green, blue);
}
