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
			setRGBColorWithValue(point.intensity(), 2);
		}
		else if (color_from_elongation_)
		{
			setRGBColorWithValue(point.elongation(), 1.5);
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
DrawableCloud::FromCloudRange(const Cloud::ConstPtr& cloud, const Eigen::Vector3f& color)
{
	return std::make_shared<DrawableCloud>(DrawableCloud(cloud, color));
}

DrawableCloud::Ptr
DrawableCloud::FromCloudIntensity(const Cloud::ConstPtr& cloud)
{
	return std::make_shared<DrawableCloud>(
			DrawableCloud(cloud, Eigen::Vector3f::Ones(), true, false));
}

DrawableCloud::Ptr
DrawableCloud::FromCloudElongation(const Cloud::ConstPtr& cloud)
{
	return std::make_shared<DrawableCloud>(
			DrawableCloud(cloud, Eigen::Vector3f::Ones(), false, true));
}

void
DrawableCloud::setRGBColorWithValue(const float& value, const float& value_max) const
{
	// https://ai.googleblog.com/2019/08/turbo-improved-rainbow-colormap-for.html
	float value_normalized = value / value_max;

	if (value_normalized < 0 || value_normalized > 1)
	{
		glColor3f(1, 1, 1);
		return;
	}

	int value_int_low = std::floor(value_normalized * 255);
	int value_int_high = std::min(255, value_int_low + 1);
	float value_fraction = value_normalized * 255 - value_int_low;

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
