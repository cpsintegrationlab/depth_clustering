// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#include <quaternion.h>

#include "visualization/viewer/viewer.h"

using std::lock_guard;
using std::mutex;

void
Viewer::AddDrawable(Drawable::Ptr drawable)
{
	lock_guard<mutex> guard(_cloud_mutex);
	_drawables.push_back(drawable);
}

void
Viewer::Clear()
{
	lock_guard<mutex> guard(_cloud_mutex);
	_drawables.clear();
}

void
Viewer::resetViewFOVDefault()
{
	setSceneCenter(qglviewer::Vec(0, 0, 0));
	setSceneRadius(range_lidar_);
	camera()->setOrientation(qglviewer::Quaternion(0, 0, -0.7071068, 0.7071068)); // Euler (x, y, z): (0, 0, -90)
	camera()->showEntireScene();
	update();
}

void
Viewer::resetViewFOVCamera()
{
	setSceneCenter(qglviewer::Vec(range_lidar_ / 2, 0, 0));
	setSceneRadius(range_lidar_ / 2);
	camera()->setOrientation(qglviewer::Quaternion(0, 0, -0.7071068, 0.7071068)); // Euler (x, y, z): (0, 0, -90)
	camera()->showEntireScene();
	update();
}

void
Viewer::draw()
{
	lock_guard<mutex> guard(_cloud_mutex);
	for (auto &drawable : _drawables)
	{
		drawable->Draw();
	}
}

void
Viewer::init()
{
	setBackgroundColor(QColor(0, 0, 0));
	resetViewFOVDefault();
	glDisable(GL_LIGHTING);
}
