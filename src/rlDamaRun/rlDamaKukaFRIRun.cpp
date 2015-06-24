/*
 * Copyright (c) 2015, Sören Jentzsch, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#include <iostream>

#include "DamaKukaFRIExecutor.h"

int main(int argc, char** argv)
{
	if(DamaKukaFRIExecutor::getInstance()->init("solution.vertices", "solution.edges"))
	{
		::std::cout << "Init succeeded." << ::std::endl;
		if(DamaKukaFRIExecutor::getInstance()->calcPath("solution.path"))
		{
			::std::cout << "calcPath succeeded." << ::std::endl;

			if(DamaKukaFRIExecutor::getInstance()->initRobot())
			{
				::std::cout << "InitRobot succeeded." << ::std::endl;

				if(DamaKukaFRIExecutor::getInstance()->followPathRobot())
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
			DamaKukaFRIExecutor::getInstance()->shutdownRobot();
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
