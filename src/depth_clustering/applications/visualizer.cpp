#include <QApplication>

#include "visualization/visualization.h"

void
printUsage(int argc, char* argv[])
{
	std::cout << std::endl << "Usage:\t" << argv[0] << std::endl;
	std::cout << "\t" << argv[0] << " [dataset segment path]" << std::endl;
	std::cout << "\t" << argv[0] << " [dataset segment path] [global config file]" << std::endl;
	std::cout << "\t" << argv[0]
			<< " [dataset segment path] [global config file] [layout config file]" << std::endl
			<< std::endl;
}

int
main(int argc, char* argv[])
{
	if (argc > 1)
	{
		if (std::string(argv[1]) == "-h")
		{
			printUsage(argc, argv);
			return 0;
		}
	}

	QApplication application(argc, argv);
	Visualization visualization;

	visualization.show();

	return application.exec();
}
