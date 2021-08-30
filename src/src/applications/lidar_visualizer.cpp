#include <QApplication>
#include <QMainWindow>

#include "visualization/visualization.h"

int
main(int argc, char* argv[])
{
	if (argc > 1)
	{
		if (std::string(argv[1]) == "-h")
		{
			std::cout << std::endl << "Usage: " << argv[0] << " [dataset path]" << std::endl
					<< std::endl;
			return 0;
		}
	}

	QApplication a(argc, argv);
	Visualization visualization;

	visualization.show();

	return a.exec();
}
