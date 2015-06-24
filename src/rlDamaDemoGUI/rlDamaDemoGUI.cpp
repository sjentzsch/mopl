/*
 * Copyright (c) 2015, Markus Rickert, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#include <QApplication>
#include <QMetaType>
#include <stdexcept>
#include <rl/math/Transform.h>
#include <rl/math/Vector.h>
#include <rl/plan/VectorList.h>

#include "MainWindow.h"
#include "DamaPlanner.h"

MainWindow* MainWindow::singleton = NULL;

int main(int argc, char** argv)
{
	try
	{
		if(argc != 2)
		{
			std::cerr << "Usage: program task.xml" << std::endl;
			return -1;
		}

		DamaPlanner::getInstance()->init(argv[1]);

		if(DamaPlanner::getInstance()->dModel->viewerMode == "off")
		{
			DamaPlanner::getInstance()->run();



			return EXIT_SUCCESS;
		}
		else
		{
			QApplication application(argc, argv);

			qRegisterMetaType< rl::math::Real >("rl::math::Real");
			qRegisterMetaType< rl::math::Transform >("rl::math::Transform");
			qRegisterMetaType< rl::math::Vector >("rl::math::Vector");
			qRegisterMetaType< rl::plan::VectorList >("rl::plan::VectorList");

			QObject::connect(&application, SIGNAL(lastWindowClosed()), &application, SLOT(quit()));

			MainWindow::instance()->show();

			int rc = application.exec();
			// cleanup Qt application here ?
			return rc;
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return -1;
	}
}
