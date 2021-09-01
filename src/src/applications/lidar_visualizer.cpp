#include <QApplication>

#include "visualization/visualization.h"

int
main(int argc, char* argv[])
{
	if (argc > 1)
	{
		if (std::string(argv[1]) == "-h")
		{
			std::cout << std::endl << "Usage:\t" << argv[0] << std::endl;
			std::cout << "\t" << argv[0] << " [dataset path]" << std::endl << std::endl;
			return 0;
		}
	}

	QApplication application(argc, argv);
	Visualization visualization;

	visualization.show();

	return application.exec();
}
