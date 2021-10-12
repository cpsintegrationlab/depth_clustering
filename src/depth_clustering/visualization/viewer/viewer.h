// Copyright Igor Bogoslavskyi, year 2017.
// In case of any problems with the code please contact me.
// Email: igor.bogoslavskyi@uni-bonn.de.

#ifndef SRC_QT_VIEWER_VIEWER_H_
#define SRC_QT_VIEWER_VIEWER_H_

#include <mutex>
#include <QGLViewer/qglviewer.h>
#include <vector>

#include "visualization/drawables/drawable.h"

class Viewer: public QGLViewer
{
public:

	explicit
	Viewer(QWidget* parent = 0) :
			QGLViewer(parent), range_lidar_(75.0)
	{
	}

	~Viewer() override
	{
	}

	void
	AddDrawable(Drawable::Ptr drawable);

	void
	Clear();

	void
	resetViewFOVDefault();

	void
	resetViewFOVCamera();

protected:

	void
	draw() override;

	void
	init() override;

private:

	const double range_lidar_;
	std::vector<Drawable::Ptr> _drawables;
	mutable std::mutex _cloud_mutex;
};

#endif  // SRC_QT_VIEWER_VIEWER_H_
