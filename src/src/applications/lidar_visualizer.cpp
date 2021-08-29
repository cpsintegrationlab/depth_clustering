#include <QApplication>
#include <QMainWindow>

#include "visualization/visualization.h"

int
main(int argc, char* argv[])
{
	QApplication a(argc, argv);
	Visualization visualization;

	visualization.show();

	return a.exec();
}
