/*
 * Copyright (c) 2015, Markus Rickert, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#include <fstream>
#include <QApplication>
#include <QDateTime>
#include <QMutexLocker>
#include <boost/lexical_cast.hpp>
#include <deque>
#include <rl/math/Quaternion.h>
#include <rl/math/Unit.h>
#include <rl/plan/Prm.h>
#include <rl/plan/Rrt.h>

#include "DamaRrt.h"
#include "DamaModel.h"
#include "DamaSampler.h"
#include "Timer.h"

#include "DamaPlanner.h"
#include "MainWindow.h"
#include "Thread.h"
#include "Viewer.h"

Thread::Thread(QObject* parent) :
	QThread(parent),
	quit(false),
	swept(false),
	running(false)
{
}

Thread::~Thread()
{
}

void
Thread::drawConfiguration(const rl::math::Vector& q)
{
	emit configurationRequested(q);
}

void
Thread::drawConfigurationEdge(const rl::math::Vector& q0, const rl::math::Vector& q1, const bool& free)
{
	emit configurationEdgeRequested(q0, q1, free);
}

void
Thread::drawConfigurationPath(const rl::plan::VectorList& path)
{
	emit configurationPathRequested(path);
}

void
Thread::drawConfigurationVertex(const rl::math::Vector& q, const bool& free)
{
	emit configurationVertexRequested(q, free);
}

void
Thread::drawLine(const rl::math::Vector& xyz0, const rl::math::Vector& xyz1)
{
	emit lineRequested(xyz0, xyz1);
}

void
Thread::drawPoint(const rl::math::Vector& xyz)
{
	emit pointRequested(xyz);
}

void
Thread::drawSphere(const rl::math::Vector& center, const rl::math::Real& radius)
{
	emit sphereRequested(center, radius);
}

void
Thread::drawSweptVolume(const rl::plan::VectorList& path)
{
	emit sweptVolumeRequested(path);
}

void
Thread::drawWork(const rl::math::Transform& t)
{
	emit workRequested(t);
}

void
Thread::drawWorkEdge(const rl::math::Vector& q0, const rl::math::Vector& q1)
{
//	emit workEdgeRequested(q0, q1);
}

void
Thread::drawWorkPath(const rl::plan::VectorList& path)
{
	emit workPathRequested(path);
}

void
Thread::drawWorkVertex(const rl::math::Vector& q)
{
//	emit workVertexRequested(q);
}

void
Thread::reset()
{
	emit resetRequested();
}

void
Thread::resetEdges()
{
	emit edgeResetRequested();
}

void
Thread::resetLines()
{
	emit lineResetRequested();
}

void
Thread::resetPoints()
{
	emit pointResetRequested();
}

void
Thread::resetSpheres()
{
	emit sphereResetRequested();
}

void
Thread::resetVertices()
{
	emit vertexResetRequested();
}

void
Thread::run()
{
	QMutexLocker lock(&MainWindow::instance()->mutex);
	this->running = true;
	::dama::Timer timer;

	if(DamaPlanner::getInstance()->dModel->viewerOnlyResult == "on")
		MainWindow::instance()->toggleView(false);
	else
		MainWindow::instance()->toggleView(true);
	
	DamaPlanner::getInstance()->run();

	if(!this->running) return;
	
	if(this->quit)
	{
		QApplication::quit();
		return;
	}
	
	MainWindow::instance()->toggleView(true);

	if(DamaPlanner::getInstance()->vecSolved.back())
	{
		if (this->swept)
		{
			this->drawSweptVolume(DamaPlanner::getInstance()->vecPath.back());
			return;
		}
		
		//this->drawConfigurationPath(path);
		rl::plan::VectorList::iterator i = DamaPlanner::getInstance()->vecPath.back().begin();
		rl::plan::VectorList::iterator j = ++(DamaPlanner::getInstance()->vecPath.back().begin());
		::std::deque< ::std::string >::iterator e = DamaPlanner::getInstance()->vecActions.back().begin();
		MainWindow::instance()->viewer->drawConfigurationVertex(*i);
		for(; i != DamaPlanner::getInstance()->vecPath.back().end() && j != DamaPlanner::getInstance()->vecPath.back().end() && e != DamaPlanner::getInstance()->vecActions.back().end(); ++i, ++j, ++e)
		{
			MainWindow::instance()->viewer->drawConfigurationVertex(*j);
			MainWindow::instance()->viewer->drawConfigurationEdge(*i, *j);
		}
		
		if (!this->running) return;
		
		if (NULL != MainWindow::instance()->optimizer)
		{
			usleep(static_cast< std::size_t >(2.0f * 1000.0f * 1000.0f));
			
			std::cout << "optimize() ... " << std::endl;;
			timer.start();
			MainWindow::instance()->optimizer->process(DamaPlanner::getInstance()->vecPath.back());
			timer.stop();
			std::cout << "optimize() " << timer.elapsed() * 1000.0f << " ms" << std::endl;
			
			this->drawConfigurationPath(DamaPlanner::getInstance()->vecPath.back());
		}
		
		rl::math::Vector inter(DamaPlanner::getInstance()->dModel->getDof());
		
		unsigned int maxLoopCount = 1;
		if(DamaPlanner::getInstance()->dModel->viewerMode == "loop")
			maxLoopCount = std::numeric_limits<unsigned int>::max();

		for(unsigned int l=0; l<maxLoopCount; l++)
		{
			//if (!this->running) break;
			
			rl::plan::VectorList::iterator i = DamaPlanner::getInstance()->vecPath.back().begin();
			rl::plan::VectorList::iterator j = ++(DamaPlanner::getInstance()->vecPath.back().begin());
			::std::deque< ::std::string >::iterator e = DamaPlanner::getInstance()->vecActions.back().begin();
			
			if (i != DamaPlanner::getInstance()->vecPath.back().end() && j != DamaPlanner::getInstance()->vecPath.back().end())
			{
				this->drawConfiguration(*i);
				usleep(static_cast< std::size_t >(1.0f * 1000.0f * 1000.0f));
			}
			
			rl::math::Real deltaSpeed = DamaPlanner::getInstance()->dModel->speedFactor * 0.0025;

			for (; i != DamaPlanner::getInstance()->vecPath.back().end() && j != DamaPlanner::getInstance()->vecPath.back().end() && e != DamaPlanner::getInstance()->vecActions.back().end(); ++i, ++j, ++e)
			{
				// Change the end-effector body according to the current primitive if needed
				dama::DamaModel* dModel2 = DamaPlanner::getInstance()->dModelVis.get();
				//MainWindow::instance()->viewer->viewer->setSceneGraph(NULL);
				::rl::sg::so::Body* myBody = NULL;
				::std::size_t indexNumber = e->find_last_of(" ");
				::std::string primNameRaw = e->substr(0,indexNumber);
				for(int i=0; i<dModel2->vecDamaPrim.size(); i++)
				{
					//::std::cout << dModel2->vecDamaPrim.at(i)->getName() << " == " << primNameRaw << " ??"  << ::std::endl;
					if(dModel2->vecDamaPrim.at(i)->getName() == primNameRaw)
					{
						myBody = dModel2->vecDamaPrim.at(i)->endEffectorBodyVis;
						break;
					}
				}
				if(myBody != dModel2->currEndEffectorBodyVis)
				{
					//::std::cout << "Viewer: change to end-effector body " << myBody->getName() << ::std::endl;
					// TODO: FIXME: crashes sometimes! if remove+add get interrupted by another Qt thread!
					dModel2->scene->getModel(0)->remove(dModel2->scene->getModel(0)->getBody(dModel2->scene->getModel(0)->getNumBodies()-1));
					dModel2->scene->getModel(0)->add(myBody);
					dModel2->currEndEffectorBodyVis = myBody;
				}
				//::std::cout << "#bodies: " << dModel2->getBodies() << ::std::endl;
				//::std::cout << "#shapes of last body '" << dModel2->getBody(dModel2->getBodies())->getName() << "': " << dModel2->getBody(dModel2->getBodies())->getNumShapes() << ::std::endl;
				//::std::cout << "#shapes of second last body '" << dModel2->getBody(dModel2->getBodies()-1)->getName() << "': " << dModel2->getBody(dModel2->getBodies()-1)->getNumShapes() << ::std::endl;
				//::rl::sg::so::Scene* sceneVis = static_cast< ::rl::sg::so::Scene* >(dModel2->scene);
				//MainWindow::instance()->viewer->viewer->setSceneGraph(sceneVis->root);

				dama::DamaModel* dModel = DamaPlanner::getInstance()->dModel.get();
				rl::math::Real steps = std::ceil(dModel->robotMovementDistance(*i, *j) / deltaSpeed);
				
				for (std::size_t k = 1; k < steps + 1; ++k)
				{
					if (!this->running) break;
					
					DamaPlanner::getInstance()->dModel->interpolate(*i, *j, k / steps, inter);
					this->drawConfiguration(inter);
					usleep(static_cast< std::size_t >(0.01f * 1000.0f * 1000.0f));
				}
			}
			
			if(l+1 < maxLoopCount)
				usleep(static_cast< std::size_t >(1.0f * 1000.0f * 1000.0f));

			/*if (!this->running) break;
			
			rl::plan::VectorList::reverse_iterator ri = path.rbegin();
			rl::plan::VectorList::reverse_iterator rj = ++path.rbegin();
			
			if (ri != path.rend() && rj != path.rend())
			{
				this->drawConfiguration(*ri);
				usleep(static_cast< std::size_t >(0.01f * 1000.0f * 1000.0f));
			}
			
			for (; ri != path.rend() && rj != path.rend(); ++ri, ++rj)
			{
				diff = *rj - *ri;
				
				rl::math::Real steps = std::ceil(DamaPlanner::getInstance()->dModel->distance(*ri, *rj) / delta);
				
				for (std::size_t k = 1; k < steps + 1; ++k)
				{
					if (!this->running) break;
					
					DamaPlanner::getInstance()->dModel->interpolate(*ri, *rj, k / steps, inter);
					this->drawConfiguration(inter);
					usleep(static_cast< std::size_t >(0.01f * 1000.0f * 1000.0f));
				}
			}*/
		}
	}

	QApplication::quit();
}

void
Thread::stop()
{
	if (this->running)
	{
		this->running = false;
		
		/*while (!this->isFinished())
		{
			QThread::usleep(0);
		}*/
	}
}
