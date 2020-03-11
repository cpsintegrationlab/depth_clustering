#include <QApplication>
#include <QMainWindow>

#include "qt/widgets/opengl_folder_player.h"

int
main(int argc, char* argv[])
{
	bool debug = false;

	if (argc > 1)
	{
		std::string mode = argv[1];

		if (mode == "debug")
		{
			debug = true;
		}
	}

	if (!debug)
	{
		__attribute__((unused)) FILE *file;
		file = freopen("/dev/null", "w", stderr);
	}

	QApplication a(argc, argv);

	OpenGlFolderPlayer folder_player_widget;
	folder_player_widget.show();

	return a.exec();
}
