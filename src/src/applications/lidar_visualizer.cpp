#include <QApplication>
#include <QMainWindow>

#include "visualization/visualization.h"

int
main(int argc, char* argv[])
{
	QApplication a(argc, argv);
	OpenGlFolderPlayer folder_player_widget;

	folder_player_widget.show();

	return a.exec();
}
