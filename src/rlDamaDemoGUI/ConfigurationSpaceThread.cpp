/*
 * Copyright (c) 2015, Markus Rickert, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#include <QMutexLocker>
#include "DamaModel.h"

#include "ConfigurationSpaceThread.h"
#include "MainWindow.h"

ConfigurationSpaceThread::ConfigurationSpaceThread(QObject* parent) :
	QThread(parent),
	delta(1.0f),
	model(NULL),
	x(0),
	y(1),
	running(false)
{
}

ConfigurationSpaceThread::~ConfigurationSpaceThread()
{
}

void
ConfigurationSpaceThread::run()
{
	QMutexLocker lock(&MainWindow::instance()->mutex);
	
	this->running = true;
	
	// TODO: fixme
	if(dama::DamaModel* model = dynamic_cast< dama::DamaModel* >(this->model))
	{
		rl::math::Vector maximum(this->model->getDof(0));
		this->model->getMaximum(maximum, 0);
		rl::math::Vector minimum(this->model->getDof(0));
		this->model->getMinimum(minimum, 0);
		
		rl::math::Real range0 = std::abs(maximum(this->x) - minimum(this->x));
		rl::math::Real range1 = std::abs(maximum(this->y) - minimum(this->y));
		
		rl::math::Real delta0 = range0 / std::ceil(range0 / this->delta);
		rl::math::Real delta1 = range1 / std::ceil(range1 / this->delta);
		
		std::size_t steps0 = static_cast< std::size_t >(std::ceil(range0 / delta0));
		std::size_t steps1 = static_cast< std::size_t >(std::ceil(range1 / delta1));
		
		rl::math::Vector q(*MainWindow::instance()->start);
		
		{
			for (std::size_t i = 0; i < steps1 + 1 && this->running; ++i)
			{
				q(this->y) = maximum(this->y) - i * delta1;
				
				for (std::size_t j = 0; j < steps0 + 1 && this->running; ++j)
				{
					q(this->x) = minimum(this->x) + j * delta0;
					
					model->setPosition(q);
					model->updateFrames();
					
					if (model->isColliding())
					{
						emit addCollision(
							q(this->x),
							q(this->y),
							delta0,
							delta1,
							0
						);
					}
				}
			}
		}
	}
	
	this->running = false;
}

void
ConfigurationSpaceThread::stop()
{
	if (this->running)
	{
		this->running = false;
		
		while (!this->isFinished())
		{
			QThread::usleep(0);
		}
	}
}
