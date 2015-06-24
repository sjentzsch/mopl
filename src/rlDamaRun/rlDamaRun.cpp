/*
 * Copyright (c) 2015, Sören Jentzsch, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#include <iostream>

#include "DamaExecutor.h"

int main(int argc, char** argv)
{
	if(DamaExecutor::getInstance()->init("solution.vertices", "solution.edges"))
	{
		::std::cout << "Init succeeded." << ::std::endl;
		if(DamaExecutor::getInstance()->calcPath("solution.path"))
		{
			::std::cout << "calcPath succeeded." << ::std::endl;

			if(DamaExecutor::getInstance()->initRobot())
			{
				::std::cout << "InitRobot succeeded." << ::std::endl;

				if(DamaExecutor::getInstance()->followPathRobot())
				{
					::std::cout << "followPathRobot succeeded." << ::std::endl;
				}
				else
				{
					::std::cerr << "followPathRobot failed." << ::std::endl;
				}
			}
			else
			{
				::std::cerr << "InitRobot failed." << ::std::endl;
			}
		}
		else
		{
			::std::cerr << "calcPath failed." << ::std::endl;
		}
	}
	else
	{
		::std::cerr << "Init failed. Could not open files." << ::std::endl;
	}

	return 0;
}
