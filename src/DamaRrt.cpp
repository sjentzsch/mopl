/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#include <chrono>
#include <boost/make_shared.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <map>
#include <deque>
#include <algorithm>

#include "DamaRrt.h"
#include "DamaModel.h"
#include "DamaSampler.h"
#include <rl/plan/Viewer.h>

#include <rl/mdl/Joint.h>
#include <Inventor/actions/SoWriteAction.h>
#include <rl/sg/so/Scene.h>
#include <rl/sg/Body.h>

#if _MSC_VER < 1600
#define nullptr NULL // TODO
#endif

namespace std
{
	std::ostream& operator<<(std::ostream &os, const rl::plan::VectorPtr q)
	{
		for (::std::size_t i = 0; i < q.get()->size(); ++i)
		{
			os << q.get()->data()[i];
			if(i < q.get()->size()-1)
				os << ", ";
		}

		return os;
	}

	std::istream& operator>>(std::istream &is, rl::plan::VectorPtr &q)
	{
		return is;
	}
}

namespace dama
{
	DamaRrt::DamaRrt(::std::size_t numMovableComponents, ::std::size_t numSupportSurfaces) :
		DamaRrtCon(),
		dModel(NULL),
		dSampler(NULL),
		randSubspace(
			::boost::mt19937(static_cast< ::boost::mt19937::result_type >(0)),
			::boost::uniform_int<>(0, numMovableComponents-1)
		),
		randSupportSurface(
			::boost::mt19937(static_cast< ::boost::mt19937::result_type >(0)),
			::boost::uniform_int<>(0, numSupportSurfaces)
		),
		extendStep(0.1f)	// set from outside manually, currently in MainWindow.cpp
	{
		this->seed(static_cast< ::boost::mt19937::result_type >(0));

		this->metaFileName = "DamaRrt-graph-meta";
		this->tree0FileName = "DamaRrt-graph-0.xml";
		this->tree1FileName = "DamaRrt-graph-1.xml";
		this->treeSolFileName = "DamaRrt-graph-sol.xml";
		this->pathSolVerticesFileName = "solution.vertices";
		this->pathSolEdgesFileName = "solution.edges";

		this->resetStatistics();
	}

	DamaRrt::~DamaRrt()
	{
	}

	::std::string DamaRrt::getName() const
	{
		return "RRT Dama";
	}

	void DamaRrt::seed(const ::boost::mt19937::result_type& value)
	{
		this->rand.engine().seed(value);
		this->randSubspace.engine().seed(value);
		this->randSupportSurface.engine().seed(value);
		if(dSampler != NULL)
			this->dSampler->seed(value);
	}

	void DamaRrt::choose(::rl::math::Vector& chosen, std::vector<bool>& currIsChosen, bool forwardSearch)
	{
		if(this->dModel->workspaceSampling)
		{
			// For Workspace drawing!
			::std::size_t currIndex2 = this->dModel->getDof(0);
			for(::std::size_t i=1; i<this->dModel->getNumMovableComponents(); ++i)
			{
				for(::std::size_t u=0; u<this->dModel->getDof(i); ++u)
				{
					chosen(currIndex2+u) = 0.0f;
					currIsChosen.at(currIndex2+u) = false;
				}
				currIndex2 += this->dModel->getDof(i);
			}
			this->dSampler->generateOnlyCollisionFree(chosen, 0, this->dModel->vecSupportSurface.size());
			for(::std::size_t u=0; u<this->dModel->getDof(0); ++u)
				currIsChosen.at(u) = true;
			return;
		}

		std::deque<std::string> dequeNames;

		::std::size_t randSubspace = 0;
		::std::size_t randSupportSurface = this->dModel->vecSupportSurface.size();
		if(this->rand() > this->dModel->sampleRobotPoseProb)
		{
			while(randSubspace == 0)
				randSubspace = this->randSubspace();
			randSupportSurface = this->randSupportSurface();

			this->dSampler->generateOnlyCollisionFree(chosen, randSubspace, randSupportSurface);
		}

		bool allObjectFree = true;

		::std::size_t currIndex = this->dModel->getDof(0);
		for(::std::size_t i=1; i<this->dModel->getNumMovableComponents(); ++i)
		{
			if(i != randSubspace)
			{
				// TODO: fix this stuff!!! I can't understand it anymore !!!

				/*if(randVal > 0.5f && forwardSearch && this->goalDimDefined->at(this->dModel->indexFromSubspace(i)))
				{
					for(::std::size_t u=0; u<this->dModel->getDof(i); ++u)
					{
						chosen(currIndex+u) = (*this->goal)(currIndex+u);
						currIsChosen.at(currIndex+u) = true;
					}

					dequeNames.push_back("goal");
					allObjectFree = false;
				}
				else if(randVal > 0.5f && !forwardSearch)
				{
					for(::std::size_t u=0; u<this->dModel->getDof(i); ++u)
					{
						chosen(currIndex+u) = (*this->start)(currIndex+u);
						currIsChosen.at(currIndex+u) = true;
					}

					dequeNames.push_back("start");
					allObjectFree = false;
				}*/
				if(this->rand() > this->dModel->sampleObjectsFreeProb)
				{
					if(this->rand() > 0.5f)
					{
						if(this->goalDimDefined->at(this->dModel->indexFromSubspace(i)))
						{
							for(::std::size_t u=0; u<this->dModel->getDof(i); ++u)
							{
								chosen(currIndex+u) = (*this->goal)(currIndex+u);
								currIsChosen.at(currIndex+u) = true;
							}

							dequeNames.push_back("goal");
							allObjectFree = false;
						}
						else
						{
							if(this->rand() > 0.5f)
							{
								for(::std::size_t u=0; u<this->dModel->getDof(i); ++u)
								{
									chosen(currIndex+u) = (*this->start)(currIndex+u);
									currIsChosen.at(currIndex+u) = true;
								}

								dequeNames.push_back("start");
								allObjectFree = false;
							}
							else
							{
								for(::std::size_t u=0; u<this->dModel->getDof(i); ++u)
								{
									chosen(currIndex+u) = 0.0f;
									currIsChosen.at(currIndex+u) = false;
								}

								dequeNames.push_back("free");
							}
						}
					}
					else
					{
						for(::std::size_t u=0; u<this->dModel->getDof(i); ++u)
						{
							chosen(currIndex+u) = (*this->start)(currIndex+u);
							currIsChosen.at(currIndex+u) = true;
						}

						dequeNames.push_back("start");
						allObjectFree = false;
					}
				}
				else
				{
					for(::std::size_t u=0; u<this->dModel->getDof(i); ++u)
					{
						chosen(currIndex+u) = 0.0f;
						currIsChosen.at(currIndex+u) = false;
					}

					dequeNames.push_back("free");
					/*if((forwardSearch && this->goalDimDefined->at(this->dModel->indexFromSubspace(i))) || !forwardSearch)
						allObjectGoal = false;*/
				}
			}
			else
			{
				for(::std::size_t u=0; u<this->dModel->getDof(i); ++u)
				{
					currIsChosen.at(currIndex+u) = true;
				}
				dequeNames.push_back("rand");
				allObjectFree = false;
			}

			currIndex += this->dModel->getDof(i);
		}

		// now determine the robot position sample values
		if(randSubspace > 0 && randSupportSurface == this->dModel->vecSupportSurface.size())
		{
			for(::std::size_t u=0; u<this->dModel->getDof(0); ++u)
				currIsChosen.at(u) = true;

			dequeNames.push_front("coupled");
		}
		else if(randSubspace == 0)
		{
			double randVal = this->rand();

			if(randVal > this->dModel->sampleRobotRandProb && forwardSearch && this->goalDimDefined->at(this->dModel->indexFromSubspace(0)))
			{
				for(::std::size_t u=0; u<this->dModel->getDof(0); ++u)
				{
					chosen(u) = (*this->goal)(u);
					currIsChosen.at(u) = true;
				}

				dequeNames.push_front("goal");
			}
			else if(randVal > this->dModel->sampleRobotRandProb && !forwardSearch)
			{
				for(::std::size_t u=0; u<this->dModel->getDof(0); ++u)
				{
					chosen(u) = (*this->start)(u);
					currIsChosen.at(u) = true;
				}

				dequeNames.push_front("start");
			}
			else
			{
				this->dSampler->generateOnlyCollisionFree(chosen, 0, this->dModel->vecSupportSurface.size());
				for(::std::size_t u=0; u<this->dModel->getDof(0); ++u)
				{
					currIsChosen.at(u) = true;
				}

				dequeNames.push_front("rand");
			}
		}
		else
		{
			for(::std::size_t u=0; u<this->dModel->getDof(0); ++u)
			{
				chosen(u) = 0.0f;
				currIsChosen.at(u) = false;
			}

			dequeNames.push_front("free");
		}

		if(this->dModel->debugMode)
		{
			std::cout << "Sample: ";
			this->dModel->printQLine(chosen, true);
			for(::std::size_t i=0; i<dequeNames.size(); ++i)
			{
				std::cout << "[" << dequeNames.at(i) << "]";
				if(i<dequeNames.size()-1)
					std::cout << ", ";
			}
			std::cout << std::endl;
		}
	}

	bool DamaRrt::solveAll(::rl::plan::VectorList& path, ::std::deque< ::std::string >& actions)
	{
		//::std::cout.precision(15);

		dama::Timer ttimer1;
		ttimer1.start();

		/*while(true)
			usleep(static_cast< std::size_t >(1.0f * 1000.0f * 1000.0f));*/

		// BEGIN TEST CODE

		// AXIS INFORMATION:
		// x-axis points from the robot to its back away from the table (top -> down)
		// y-axis points from the robot to the right along the table (left -> right)

		// ATTENTION: following code needs "MainWindow::instance()->toggleView(true);" in Thread.cpp

		/*rl::math::Vector qTemp(this->dModel->mdl->getDof());
		rl::math::Vector q_ik_start;
		rl::math::Vector dirVecMovingObject(2);
		rl::math::Transform x_grasp, x_in, x_out;
		rl::math::Vector posObject(3);

		posObject(0) = 0.0;
		posObject(1) = 0.2;
		posObject(2) = 0.146;// + DamaPrimPickup::HEIGHT_PICKUP_DIST_OBJECT;
		(*this->start)(15) = posObject(2);
		bool doGrasp = true;

		::rl::mdl::Dynamic* currMdl;
		if(doGrasp)
			currMdl = this->dModel->mdlGrasp;
		else
			currMdl = this->dModel->mdl;
		double theta, theta_in, theta_out;
		double thetaDeg = 0.0;
		//for(; posObject(1)<this->dModel->vecSupportSurface.at(1)->getMax(1); posObject(1)+=0.001)
		for(thetaDeg=0.0; thetaDeg<2*360.0; thetaDeg+=1)
		{
			if(doGrasp)
			{
				::std::cout << "#### position object: " << posObject(0) << " | " << posObject(1) << " | " << posObject(2) << ::std::endl;
			}
			else
			{
				dirVecMovingObject(0) = -sin(thetaDeg * rl::math::DEG2RAD);
				dirVecMovingObject(1) = cos(thetaDeg * rl::math::DEG2RAD);
				::std::cout << "#### dirVecMovingObject(0): " << dirVecMovingObject(0) << ::std::endl;
				::std::cout << "#### dirVecMovingObject(1): " << dirVecMovingObject(1) << ::std::endl;
			}

			// FOR GRASPING
			x_grasp.translation().x() = posObject(0);
			x_grasp.translation().y() = posObject(1);
			x_grasp.translation().z() = posObject(2) + DamaPrimPickup::HEIGHT_OFFSET_OBJECT;
			x_grasp.linear() = (
				rl::math::AngleAxis(45 * rl::math::DEG2RAD, rl::math::Vector3::UnitZ()) *	// WILL BE FREE AXIS
				rl::math::AngleAxis(90 * rl::math::DEG2RAD, rl::math::Vector3::UnitY()) *
				rl::math::AngleAxis(180 * rl::math::DEG2RAD, rl::math::Vector3::UnitX())
			).toRotationMatrix();

			// calculate theta for pushing configuration
			theta = atan(- dirVecMovingObject(0) / dirVecMovingObject(1));
			theta_in = theta;
			if(dirVecMovingObject(1) >= 0.0)		// for pushing with inside surface
				theta_in += M_PI;
			theta_out = theta;
			if(dirVecMovingObject(1) < 0.0)		// for pushing with outside surface
				theta_out += M_PI;

			if(!doGrasp)
			{
				// TODO: following two maybe not needed -> test!
				//if(theta < 0)
				//	theta += 2*M_PI;
				//if(theta >= 2*M_PI)
				//	theta -= 2*M_PI;
				//::std::cout << "#### theta (rad): " << (theta) << ::std::endl;
				::std::cout << "#### theta (deg): " << (theta * rl::math::RAD2DEG) << ::std::endl;
				//::std::cout << "#### theta_in (rad): " << (theta_in) << ::std::endl;
				::std::cout << "#### theta_in (deg): " << (theta_in * rl::math::RAD2DEG) << ::std::endl;
				//::std::cout << "#### theta_out (rad): " << (theta_out) << ::std::endl;
				::std::cout << "#### theta_out (deg): " << (theta_out * rl::math::RAD2DEG) << ::std::endl;
			}

			// FOR PUSHING (with the inside surface)
			x_in.translation().x() = posObject(0) - DamaPrimPush::DIST_TO_OBJECT_XY*sin(theta_in);
			x_in.translation().y() = posObject(1) + DamaPrimPush::DIST_TO_OBJECT_XY*cos(theta_in);
			x_in.translation().z() = posObject(2) + DamaPrimPush::HEIGHT_OFFSET_OBJECT;
			x_in.linear() = (
				rl::math::AngleAxis(theta_in, rl::math::Vector3::UnitZ()) *
				rl::math::AngleAxis(-15 * rl::math::DEG2RAD, rl::math::Vector3::UnitX()) *
				rl::math::AngleAxis(75 * rl::math::DEG2RAD, rl::math::Vector3::UnitY()) *
				rl::math::AngleAxis(180 * rl::math::DEG2RAD, rl::math::Vector3::UnitX())
			).toRotationMatrix();

			// FOR PUSHING (with the outside surface)
			x_out.translation().x() = posObject(0) - DamaPrimPush::DIST_TO_OBJECT_XY*sin(theta_in);
			x_out.translation().y() = posObject(1) + DamaPrimPush::DIST_TO_OBJECT_XY*cos(theta_in);
			x_out.translation().z() = posObject(2) + DamaPrimPush::HEIGHT_OFFSET_OBJECT;
			x_out.linear() = (
				rl::math::AngleAxis(theta_out, rl::math::Vector3::UnitZ()) *
				rl::math::AngleAxis(15 * rl::math::DEG2RAD, rl::math::Vector3::UnitX()) *
				rl::math::AngleAxis(75 * rl::math::DEG2RAD, rl::math::Vector3::UnitY()) *
				rl::math::AngleAxis(180 * rl::math::DEG2RAD, rl::math::Vector3::UnitX())
			).toRotationMatrix();

			//this->dModel->printTransform(x);

			rl::math::Vector qt(currMdl->getDof());
			this->dModel->updateRobotPosition(qt, currMdl);

			rl::math::Matrix66 constraint_position_orient = rl::math::Matrix66::Zero();

			// Attention: q_ik_start must be set to an initial value after each call of inversePositionTaskPosture (because it changes within the function)
			q_ik_start = (rl::math::Vector(10) << 0.0, 0.0, 0.0, 0.0, 15.0 * rl::math::DEG2RAD, -20.0 * rl::math::DEG2RAD, 12.0 * rl::math::DEG2RAD, 112.0 * rl::math::DEG2RAD, -10.0 * rl::math::DEG2RAD, 0.0).finished();

			::rl::util::Timer ttimer2;
			ttimer2.start();
			if(doGrasp)
			{
				constraint_position_orient.diagonal() << 1, 1, 1, 1, 1, 0;

				if(this->dModel->debugMode)
				{
					::std::cout << "Calculate IK1 to T = (";
					this->dModel->printTransform(x_grasp, false);
					::std::cout << ") starting at q_deg = (";
					this->dModel->printQLine(q_ik_start * rl::math::RAD2DEG, false, false, false, false);
					::std::cout << ")" << ::std::endl;
				}

				if(DamaModel::calcInversePositionTaskPosture(currMdl, x_grasp, q_ik_start, constraint_position_orient, this->dModel->coupleJoint1And2))
				{
					ttimer2.stop();
					//::std::cout << "x_grasp: worked!" << ::std::endl;
				}
				else
				{
					//::std::cout << "grasp IK did not work!" << ::std::endl;
					currMdl->setPosition(qt);
				}
			}
			else
			{
				constraint_position_orient.diagonal() << 1, 1, 1, 1, 1, 1;

				if(DamaModel::calcInversePositionTaskPosture(currMdl, x_in, q_ik_start, constraint_position_orient, this->dModel->coupleJoint1And2))
				{
					ttimer2.stop();
					//::std::cout << "ttimer2: " << ttimer2.elapsed()*1000 << ::std::endl;
					//::std::cout << "x_in: worked!" << ::std::endl;
				}
				else
				{
					q_ik_start = (rl::math::Vector(10) << 0.0, 0.0, 0.0, 0.0, 15.0 * rl::math::DEG2RAD, -20.0 * rl::math::DEG2RAD, 12.0 * rl::math::DEG2RAD, 112.0 * rl::math::DEG2RAD, -10.0 * rl::math::DEG2RAD, 0.0).finished();
					if(DamaModel::calcInversePositionTaskPosture(currMdl, x_out, q_ik_start, constraint_position_orient, this->dModel->coupleJoint1And2))
					{
						//::std::cout << "x_out: worked!" << ::std::endl;
					}
					else
					{
						//::std::cout << "both did not work!" << ::std::endl;
						currMdl->setPosition(qt);
					}
				}
			}

			this->dModel->updateRobotPosition(qTemp, currMdl);
			rl::math::Transform x2 = this->dModel->calcForwardKinematics(qTemp, 0);
			this->dModel->printQLine(qTemp, false, true, true, false);
			this->dModel->printQLine(qTemp, false, true, true, true);
			this->dModel->printTransform(x2);

			ttimer1.stop();
			//::std::cout << "ttimer1: " << ttimer1.elapsed()*1000 << ::std::endl;

			rl::math::Vector qDraw(this->dModel->getDof());
			for(::std::size_t i=0; i<currMdl->getDof(); ++i)
				qDraw(i) = qTemp(i);
			for(::std::size_t i=currMdl->getDof(); i<this->dModel->getDof(); ++i)
				qDraw(i) = (*this->start)(i);
			//this->dModel->printQLine(qDraw, false, true, true, true);
			this->dModel->updateFrames();
			this->viewer->drawConfiguration(qDraw);

			this->dModel->checkCollisionRobotObjects = false;
			if(this->dModel->isColliding(DamaPrimTransit::getInstance()->getName()))
				::std::cerr << "DOES COLLIDE!" << ::std::endl;

			usleep(static_cast< std::size_t >(0.01f * 1000.0f * 1000.0f));
		}
		while(true);
		/* */

		path.clear();
		actions.clear();
		bool singleSolved;
		if(!this->hierarchicalVersion)
		{
			if(this->dModel->debugMode)
				::std::cout << "Solving with flat version ... " << ::std::endl;
			this->dModel->checkCollisionRobotObjects = true;
			singleSolved = solve();

			if(!this->dModel->showFullTree)
			{
				if(this->viewer != NULL)
					this->viewer->reset();
			}

			if(singleSolved)
			{
				this->getPath(path, actions);
				this->postProc(path, actions);
			}
			this->edgeCount = this->getNumEdges();
			this->vertexCount = this->getNumVertices();

			return singleSolved;
		}
		else
		{
			if(this->dModel->debugMode)
				::std::cout << "Solving with hierarchical version ... " << ::std::endl;
			this->duration = ::std::chrono::duration_cast< ::std::chrono::steady_clock::duration >( ::std::chrono::duration< double >(this->dModel->objectPathTimeout));
			this->dModel->checkCollisionRobotObjects = false;
			singleSolved = solve();
			if(!singleSolved)
				return false;
		}

		this->dModel->checkCollisionRobotObjects = true;
		this->duration = ::std::chrono::duration_cast< ::std::chrono::steady_clock::duration >( ::std::chrono::duration< double >(this->dModel->subProblemTimeout) );
		::std::size_t maxTries = 1;

		/*rl::plan::VectorList pathRe;
		::std::deque< ::std::string > actionsRe;
		this->getPath(pathRe, actionsRe);
		rl::plan::VectorList::iterator i3 = pathRe.begin();
		rl::plan::VectorList::iterator j3 = ++pathRe.begin();
		::std::deque< ::std::string >::iterator e3 = actionsRe.begin();
		this->dModel->printQLine(*i3);
		for(; i3 != pathRe.end() && j3 != pathRe.end() && e3 != actionsRe.end(); ++i3, ++j3, ++e3)
		{
			::std::cout << *e3 << ::std::endl;
			this->dModel->printQLine(*j3);
		}*/

		rl::plan::VectorList objectPath;
		this->getObjectPath(objectPath);
		this->edgeCount = this->getNumEdges();
		this->vertexCount = this->getNumVertices();

		bool transitToRobotGoal = false;
		::rl::math::Vector goalFinal = *(this->goal);
		::std::vector<bool> goalFinalDimDefined(*(this->goalDimDefined));
		for(::std::size_t i=0; i<this->dModel->getDof(0); ++i)
		{
			if((*this->goalDimDefined).at(i))
			{
				transitToRobotGoal = true;
				break;
			}
		}

		rl::plan::VectorList robotPathSingle;
		::std::deque< ::std::string > robotActionsSingle;
		rl::plan::VectorList::iterator i = objectPath.begin();
		rl::plan::VectorList::iterator j = ++objectPath.begin();
		if(this->dModel->debugMode)
		{
			std::cout << "Object Path:" << ::std::endl;
			std::cout << ">> ";
			this->dModel->printQLine(*i);
			for(; i != objectPath.end() && j != objectPath.end(); ++i, ++j)
			{
				std::cout << ">> ";
				this->dModel->printQLine(*j);
			}
		}

		for(i = objectPath.begin(), j = ++objectPath.begin(); i != objectPath.end() && j != objectPath.end(); ++i, ++j)
		{
			::rl::math::Vector startTemp;
			if(i == objectPath.begin())
				startTemp = *i;
			else
				startTemp = robotPathSingle.back();
			::rl::math::Vector goalTemp = *j;
			// set all object goal dim defines to true
			for(::std::size_t k=0; k<this->dModel->getDof(); ++k)
				(*this->goalDimDefined).at(k) = true;
			// robot pose does not matter for the next goal
			for(::std::size_t k=0; k<this->dModel->getDof(0); ++k)
			{
				(*this->goalDimDefined).at(k) = false;
				goalTemp(k) = 0.0;
			}
			this->start = &startTemp;
			this->goal = &goalTemp;

			::std::size_t countTrySingle = 0;
			do
			{
				this->reset();

				countTrySingle++;

				if(this->dModel->debugMode)
				{
					::std::cout << "Solving from < ";
					this->dModel->printQLine(*this->start, false, false);
					::std::cout << " > to < ";
					this->dModel->printQLine(*this->goal, false, false);
					::std::cout << " > (Attempt Nr. " << countTrySingle << ")" << ::std::endl;
				}

				singleSolved = solve();

			} while(!singleSolved && countTrySingle < maxTries);
			if(!singleSolved)
				return false;
			this->edgeCount += this->getNumEdges();
			this->vertexCount += this->getNumVertices();

			robotPathSingle.clear();
			robotActionsSingle.clear();
			this->getPath(robotPathSingle, robotActionsSingle);
			path.insert(path.end(), robotPathSingle.begin(), robotPathSingle.end());
			actions.insert(actions.end(), robotActionsSingle.begin(), robotActionsSingle.end());

			// one action is missing for each hierarchical step -> insert DamaPrimTransit manually
			if((boost::next(i) != objectPath.end() && boost::next(j) != objectPath.end()) || transitToRobotGoal)
				actions.push_back(DamaPrimTransit::getInstance()->getName());
		}
		if(transitToRobotGoal)
		{
			if(i == objectPath.begin())
				this->start = &(*i);
			else
				this->start = &(path.back());
			(*this->goalDimDefined) = goalFinalDimDefined;
			this->goal = &goalFinal;

			::std::size_t countTrySingle = 0;
			do
			{
				this->reset();

				countTrySingle++;

				if(this->dModel->debugMode)
				{
					::std::cout << "Solving from < ";
					this->dModel->printQLine(*this->start, false, false);
					::std::cout << " > to < ";
					this->dModel->printQLine(*this->goal, false, false);
					::std::cout << " > (Attempt Nr. " << countTrySingle << ")" << ::std::endl;
				}

				singleSolved = solve();

			} while(!singleSolved && countTrySingle < maxTries);
			if(!singleSolved)
				return false;
			this->edgeCount += this->getNumEdges();
			this->vertexCount += this->getNumVertices();

			robotPathSingle.clear();
			robotActionsSingle.clear();
			this->getPath(robotPathSingle, robotActionsSingle);
			path.insert(path.end(), robotPathSingle.begin(), robotPathSingle.end());
			actions.insert(actions.end(), robotActionsSingle.begin(), robotActionsSingle.end());
		}

		if(this->viewer != NULL)
			this->viewer->reset();

		this->postProc(path, actions);

		return true;
	}

	bool DamaRrt::solve()
	{
		// pause so I can capture screen
		//std::cout << "Notice: Sleeping 40 sec so you can start screencasting." << std::endl;
		//rl::util::Timer::sleep(40.0);

		// write meta file -> corresponds to unfinished or unsuccessful solving procedure
		std::ofstream myfile;
		myfile.open(metaFileName.c_str());
		myfile << this->twoTreeVersion << std::endl;
		myfile.close();

		this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared< ::rl::math::Vector >(*this->start));
		Tree* a = &this->tree[0];
		Tree* b;

		if(this->twoTreeVersion)
		{
			//this->begin[1] = this->addVertex(this->tree[1], ::boost::make_shared< ::rl::math::Vector >(*this->goal));
			b = &this->tree[1];

			// TODO: part goal option 1 [mix with option 2 ?!]
			/*::rl::math::Vector newGoalSample(*(this->goal));
			for(::std::size_t i=0; i<1000; ++i)
			{
				// TODO: fixed permutation count for each start-pose (blabla, just think how to improve this one!)
				this->sampleFromGoalFull(newGoalSample);
				this->addVertex(*b, ::boost::make_shared< ::rl::math::Vector >(newGoalSample));
				this->dModel->printQLine(newGoalSample, false, true);
			}*/
		}

		::rl::math::Vector samplePart(this->dModel->getDof());
		::rl::math::Vector sampleFull(this->dModel->getDof());
		::rl::math::Vector goalFull(this->dModel->getDof());
		this->dModel->currIsChosen.resize(this->dModel->getDof());
		this->dModel->forwardSearch = true;

		::std::size_t countWriteGraphML = 1;
		dama::Timer timerSample;
		dama::Timer timerSamplePart;
		::std::vector<double> timeSample(8, 0);
		::std::vector<double> timeSampleGlobal(8, 0);
		::std::vector<double> timeSampleGlobalAverage(8, 0);
		::std::size_t countSample = 0;

		timer.start();
		timer.stop();

		while (timer.elapsedDuration() < this->duration)
		{
			// assure that forward/backward-search and a/b are consistent
			if(!this->twoTreeVersion)
				assert(this->dModel->forwardSearch && a == &this->tree[0] && b == NULL);
			else
				assert((this->dModel->forwardSearch && &this->tree[0] == a && &this->tree[1] == b) || (!this->dModel->forwardSearch && &this->tree[0] == b && &this->tree[1] == a));

			timerSample.start();

			// Take new sample
			if(this->dModel->debugMode)
				std::cout << "# Take new sample " << (this->dModel->forwardSearch ? "(forwardSearch)" : "(backwardSearch)") << std::endl;
			timerSamplePart.start();
			samplePart.setZero();
			this->choose(samplePart, this->dModel->currIsChosen, this->dModel->forwardSearch);
			timerSamplePart.stop();
			timeSample[1] = timerSamplePart.elapsed();

			// Determine nearest neighbor to new sample in tree a
			timerSamplePart.start();
			Neighbor aNearest = this->nearest(*a, samplePart);
			// TODO: part goal option 2
			// In case of backward-search, test if sample lies nearer to the partially defined goal than to the nearest neighbor
			if(!this->dModel->forwardSearch)
			{
				this->completePartialGoal(samplePart, goalFull);

				::rl::math::Real d = this->dModel->distance(goalFull, samplePart);

				if(this->dModel->debugMode)
				{
					std::cout << "Test distance to goal: ";
					this->dModel->printQLine(goalFull, false, false);
					std::cout << " [distance: " << d << "]" << std::endl;
				}

				if(d < aNearest.second)
				{
					if(d > this->dModel->epsilon)
						aNearest.first = this->addVertex(*a, ::std::make_shared< ::rl::math::Vector >(goalFull));
					aNearest.second = d;
				}
			}
			if(aNearest.first == NULL)
				::std::cerr << "ATTENTION: No nearest neighbor found: aNearest.first == NULL with distance " << aNearest.second << ::std::endl;
			timerSamplePart.stop();
			timeSample[2] = timerSamplePart.elapsed();
			if(this->dModel->debugMode && aNearest.first != NULL)
			{
				std::cout << "Nearest neighbor (a): ";
				this->dModel->printQLine(*((*a)[aNearest.first].q.get()), false, false);
				std::cout << " [index: " << (*a)[aNearest.first].index << " | distance: " << aNearest.second << "]" << std::endl;
			}
			// Resume with new sample if distance was nearly 0 or max
			if(aNearest.second < this->dModel->epsilon || aNearest.second == ::std::numeric_limits< ::rl::math::Real >::max())
			{
				if(this->dModel->debugMode)
				{
					if(aNearest.second < this->dModel->epsilon)
						std::cout << "=> Sample and nearest neighbor are identical. Resume with new sample." << std::endl;
					else
						std::cout << "=> Sample is unreachable from the nearest neighbor. Resume with new sample." << std::endl;
					std::cout << std::endl << std::endl;
				}

				timer.stop();
				continue;
			}

			// Complete the sample with respect to the nearest neighbor
			this->dModel->completePartialSample(samplePart, *(*a)[aNearest.first].q, sampleFull);
			if(this->dModel->debugMode)
			{
				std::cout << "Sample full (a): ";
				this->dModel->printQLine(sampleFull, false, true);
			}

			// Propagate with FORWARD-control between the nearest neighbor and the sample - result: list of configuration points
			timerSamplePart.start();
			::std::vector< ::rl::math::Vector > vecRes;
			::std::vector< ::std::string > vecEdgeAction;
			ESP::Res res = this->dModel->emptySpacePlanner(vecRes, *(*a)[aNearest.first].q, sampleFull, vecEdgeAction, true);
			timerSamplePart.stop();
			timeSample[3] = timerSamplePart.elapsed();
			if(this->dModel->debugMode)
			{
				std::cout << "Empty Space Planner (ESP) - Result: " << ESP::cRes[res] << std::endl;
				std::cout << "List of Configuration Points (NN -> Sample): " << std::endl;
				for(::std::size_t u=0; u<vecRes.size(); ++u)
				{
					std::cout << ">> ";
					this->dModel->printQLine(vecRes.at(u), false, false);
					if(u < vecRes.size()-1)
						std::cout << " [-> " << vecEdgeAction.at(u) << "]";
					std::cout << std::endl;
				}
			}
			// Sample is now completed and propagate might have skipped the last Transit -> currIsChosen to all true
			for(::std::size_t i=0; i<this->dModel->getDof(); ++i)
				this->dModel->currIsChosen.at(i) = true;

			// Connect the nearest neighbor with the sample (via the list of configuration points) in tree a
			timerSamplePart.start();
			Vertex aConnected = NULL;
			bool aReached = false;
			// Only connect if ESP succeeded or it found that the goal is unreachable, but we start with nearestNeighbor (i.e. we connect forwards), so that we can quit at any time
			if(res == ESP::OK || (res == ESP::UNREACHABLE && this->dModel->forwardSearch))
			{
				for(::std::size_t u=0; u<vecRes.size()-1; ++u)
				{
					aReached = this->connect(*a, aNearest, vecRes.at(u+1), vecEdgeAction.at(u), aConnected);

					if(!aReached)
					{
						if(this->dModel->debugMode)
							std::cout << "Collision occurred during 'connect' in tree a." << std::endl;
						break;
					}

					aNearest = Neighbor(aConnected, ::std::numeric_limits< ::rl::math::Real >::max());
				}
			}
			if(!this->twoTreeVersion && aReached && this->dModel->isGoal(*((*a)[aConnected].q)))
			{
				this->finalizeWork(&aConnected, NULL);
				return true;
			}
			timerSamplePart.stop();
			timeSample[4] = timerSamplePart.elapsed();

			timeSample[5] = 0;
			timeSample[6] = 0;
			timeSample[7] = 0;
			if(this->twoTreeVersion)
			{
				// swap forward/backward-Search
				this->dModel->forwardSearch = !this->dModel->forwardSearch;

				if(aConnected != NULL)
				{
					if(this->dModel->debugMode)
						std::cout << "# Init second phase " << (this->dModel->forwardSearch ? "(forwardSearch)" : "(backwardSearch)") << std::endl;

					// Determine nearest neighbor to aConnected in tree b
					timerSamplePart.start();
					Neighbor bNearest = this->nearest(*b, *(*a)[aConnected].q);
					// TODO: part goal option 2
					// In case of backward-search, test if aConnected lies nearer to the partially defined goal than to the nearest neighbor
					if(!this->dModel->forwardSearch)
					{
						this->completePartialGoal(*(*a)[aConnected].q, goalFull);

						::rl::math::Real d = this->dModel->distance(goalFull, *(*a)[aConnected].q);

						if(this->dModel->debugMode)
						{
							std::cout << "Test distance to goal: ";
							this->dModel->printQLine(goalFull, false, false);
							std::cout << " [distance: " << d << "]" << std::endl;
						}

						if(d < bNearest.second)
						{
							bNearest.first = this->addVertex(*b, ::std::make_shared< ::rl::math::Vector >(goalFull));
							bNearest.second = d;
						}
					}
					timerSamplePart.stop();
					timeSample[5] = timerSamplePart.elapsed();
					if(this->dModel->debugMode)
					{
						std::cout << "Nearest neighbor (b): ";
						this->dModel->printQLine(*((*b)[bNearest.first].q.get()), false, false);
						std::cout << " [index: " << (*b)[bNearest.first].index << " | distance: " << bNearest.second << "]" << std::endl;
					}
					// Resume with new sample if distance max, finish if distance nearly 0
					if(bNearest.second < this->dModel->epsilon)
					{
						if(this->dModel->debugMode)
							std::cout << "=> Sample and nearest neighbor are identical. Finished." << std::endl << std::endl << std::endl;

						this->finalizeWork(&this->tree[0] == a ? &aConnected : &(bNearest.first), &this->tree[1] == b ? &(bNearest.first) : &aConnected);
						return true;
					}
					else if(bNearest.second == ::std::numeric_limits< ::rl::math::Real >::max())
					{
						if(this->dModel->debugMode)
							std::cout << "=> Sample is unreachable from the nearest neighbor. Resume with new sample." << std::endl << std::endl << std::endl;

						timer.stop();

						// swap forward/backward-Search to start over again!
						this->dModel->forwardSearch = !this->dModel->forwardSearch;

						continue;
					}

					// Propagate with FORWARD-control between aConnected and the nearest neighbor - result: list of configuration points
					timerSamplePart.start();
					::std::vector< ::rl::math::Vector > vecRes;
					::std::vector< ::std::string > vecEdgeAction;
					ESP::Res res = this->dModel->emptySpacePlanner(vecRes, *(*b)[bNearest.first].q, *(*a)[aConnected].q, vecEdgeAction, true);
					timerSamplePart.stop();
					timeSample[6] = timerSamplePart.elapsed();
					if(this->dModel->debugMode)
					{
						std::cout << "Empty Space Planner (ESP) - Result: " << ESP::cRes[res] << std::endl;
						std::cout << "List of Configuration Points (NN -> Sample): " << std::endl;
						for(::std::size_t u=0; u<vecRes.size(); ++u)
						{
							std::cout << ">> ";
							this->dModel->printQLine(vecRes.at(u), false, false);
							if(u < vecRes.size()-1)
								std::cout << " [-> " << vecEdgeAction.at(u) << "]";
							std::cout << std::endl;
						}
					}

					// Connect the nearest neighbor with aConnected (via the list of configuration points in REVERSE order) in tree b
					timerSamplePart.start();
					Vertex bConnected = NULL;
					bool bReached = false;
					// Only connect if ESP succeeded or it found that the goal is unreachable, but we start with nearestNeighbor (i.e. we connect forwards), so that we can quit at any time
					if(res == ESP::OK || (res == ESP::UNREACHABLE && this->dModel->forwardSearch))
					{
						for(::std::size_t u=0; u<vecRes.size()-1; ++u)
						{
							bReached = this->connect(*b, bNearest, vecRes.at(u+1), vecEdgeAction.at(u), bConnected);

							if(!bReached)
							{
								if(this->dModel->debugMode)
									std::cout << "Collision occurred during 'connect' in tree b." << std::endl;
								break;
							}

							bNearest = Neighbor(bConnected, ::std::numeric_limits< ::rl::math::Real >::max());
						}
					}
					if(bReached && !this->dModel->isMoving(*(*a)[aConnected].q, *(*b)[bConnected].q))
					{
						this->finalizeWork(&this->tree[0] == a ? &aConnected : &bConnected, &this->tree[1] == b ? &bConnected : &aConnected);
						return true;
					}
					timerSamplePart.stop();
					timeSample[7] = timerSamplePart.elapsed();
				}

				// Swap trees
				::std::swap(a, b);
			}

			// Print some final statistics
			timer.stop();
			timerSample.stop();
			timeSample[0] = timerSample.elapsed();
			this->iterationCount++;
			countSample++;
			for(::std::size_t i=0; i<timeSample.size(); ++i)
			{
				timeSampleGlobal[i] += timeSample[i];
				timeSampleGlobalAverage[i] = timeSampleGlobal[i]/countSample;
			}

			if(this->dModel->workspaceSampling)
			{
				if(this->getNumEdges() >= this->dModel->workspaceSamplingEdges)
					while(true);
			}

			this->timeSampling += timeSample[1];
			this->timeNNSearch += timeSample[2] + timeSample[5];
			this->timePropagate += timeSample[3] + timeSample[6];
			this->timeConnect += timeSample[4] + timeSample[7];

			if(this->dModel->debugMode)
			{
				std::cout << "Edges: " << this->getNumEdges() << (this->twoTreeVersion ? ::std::string(" (") + boost::lexical_cast<std::string>(::boost::num_edges(this->tree[0])) + ::std::string(", ") + boost::lexical_cast<std::string>(::boost::num_edges(this->tree[1])) + ::std::string(")") : "") << " | Vertices: " << this->getNumVertices() << (this->twoTreeVersion ? ::std::string(" (") + boost::lexical_cast<std::string>(::boost::num_vertices(this->tree[0])) + ::std::string(", ") + boost::lexical_cast<std::string>(::boost::num_vertices(this->tree[1])) + ::std::string(")") : "") << " | Sample Nr. " << countSample << " | Global Time: " << timer.elapsed() << " | IK Time Global: " << this->dModel->timeIK << " | Sample Time Global: " << timeSampleGlobal[0] << std::endl;
				std::cout << "Sample Time: " << timeSample[0] << " | 1) sample: " << (timeSample[1]/timeSample[0]*100) << "%" << " | 2) nn a: " << (timeSample[2]/timeSample[0]*100) << "%" << " | 3) propagate a: " << (timeSample[3]/timeSample[0]*100) << "%" << " | 4) connect a: " << (timeSample[4]/timeSample[0]*100) << "%" << " | 5) nn b: " << (timeSample[5]/timeSample[0]*100) << "%" << " | 6) propagate b: " << (timeSample[6]/timeSample[0]*100) << "%" << " | 7) connect b: " << (timeSample[7]/timeSample[0]*100) << "%" << " | -> all: " << ((timeSample[1]+timeSample[2]+timeSample[3]+timeSample[4]+timeSample[5]+timeSample[6]+timeSample[7])/timeSample[0]*100) << "%" << std::endl;
				std::cout << "Average: " << timeSampleGlobalAverage[0] << " | 1) sample: " << (timeSampleGlobalAverage[1]/timeSampleGlobalAverage[0]*100) << "%" << " | 2) nn a: " << (timeSampleGlobalAverage[2]/timeSampleGlobalAverage[0]*100) << "%" << " | 3) propagate a: " << (timeSampleGlobalAverage[3]/timeSampleGlobalAverage[0]*100) << "%" << " | 4) connect a: " << (timeSampleGlobalAverage[4]/timeSampleGlobalAverage[0]*100) << "%" << " | 5) nn b: " << (timeSampleGlobalAverage[5]/timeSampleGlobalAverage[0]*100) << "%" << " | 6) propagate b: " << (timeSampleGlobalAverage[6]/timeSampleGlobalAverage[0]*100) << "%" << " | 7) connect b: " << (timeSampleGlobalAverage[7]/timeSampleGlobalAverage[0]*100) << "%" << " | -> all: " << ((timeSampleGlobalAverage[1]+timeSampleGlobalAverage[2]+timeSampleGlobalAverage[3]+timeSampleGlobalAverage[4]+timeSampleGlobalAverage[5]+timeSampleGlobalAverage[6]+timeSampleGlobalAverage[7])/timeSampleGlobalAverage[0]*100) << "%" << std::endl;
				std::cout << "Average Steps: " << "sample: " << (timeSampleGlobalAverage[1]/timeSampleGlobalAverage[0]*100) << "%" << " | nn: " << ((timeSampleGlobalAverage[2]+timeSampleGlobalAverage[5])/timeSampleGlobalAverage[0]*100) << "%" << " | propagate: " << ((timeSampleGlobalAverage[3]+timeSampleGlobalAverage[6])/timeSampleGlobalAverage[0]*100) << "%" << " | connect: " << ((timeSampleGlobalAverage[4]+timeSampleGlobalAverage[7])/timeSampleGlobalAverage[0]*100) << "%" << std::endl;
				std::cout << std::endl << std::endl;
			}

			// write the GraphML file after each 5000 samples
			if(countSample >= countWriteGraphML*5000)
			{
				this->writeGraphML(this->tree[0], tree0FileName);
				if(this->twoTreeVersion)
					this->writeGraphML(this->tree[1], tree1FileName);
				countWriteGraphML++;
			}


			// TODO: a bit buggy, but countEqualNodes-value should be 0 in the end ...
			/*::std::size_t indexI = 0;
			::std::size_t countEqualNodes = 0;
			for(VertexIteratorPair i = ::boost::vertices(this->tree[0]); i.first != i.second; ++i.first)
			{
				for(VertexIteratorPair j = ::boost::vertices(this->tree[0]); j.first != j.second; ++j.first)
				{
					bool incrementedOnce = false;
					for(::std::size_t w=0; w<indexI+1 && j.first != j.second; ++w)
					{
						++j.first;
						incrementedOnce = true;
					}
					if(!incrementedOnce || j.first == j.second)
						break;
					::rl::math::Vector qt1 = *this->tree[0][*i.first].q;
					::rl::math::Vector qt2 = *this->tree[0][*j.first].q;
					bool isEqual = true;
					for(::std::size_t t = 0; t<qt1.size(); ++t)
					{
						if(qt1(t) != qt2(t))
						{
							isEqual = false;
							break;
						}
					}
					if(isEqual)
						countEqualNodes++;
				}
				indexI++;
			}
			std::cout << "*******+****+** -> countEqualNodes: " << countEqualNodes << std::endl;*/


			/*std::cout << "Edge list:" << std::endl;
			for(VertexIteratorPair i = ::boost::vertices(this->tree[0]); i.first != i.second; ++i.first)
			{
				std::cout << this->tree[0][*i.first].index << ": ";
				this->dModel->printQLine(*this->tree[0][*i.first].q);
			}

			for(EdgeIteratorPair i = ::boost::edges(this->tree[0]); i.first != i.second; ++i.first)
			{
				Edge e = *i.first;
				Vertex u = ::boost::source(e, this->tree[0]);
				Vertex v = ::boost::target(e, this->tree[0]);
				std::cout << "[";
				this->dModel->printQLine(*this->tree[0][u].q, false, false);
				std::cout << "] to [";
				this->dModel->printQLine(*this->tree[0][v].q, false, false);
				std::cout << "]" << std::endl;
			}*/

			// TODO: delete me if no longer waiting for ENTER-Key needed
			//while(std::cin.get()!='\n');

			timer.stop();
		}

		return false;
	}

	bool DamaRrt::connectOld(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen, ::std::string edgeAction, Vertex& vertex)
	{
		::rl::math::Real step = this->delta;

		::rl::math::Real distance = this->dModel->robotMovementDistance(*tree[nearest.first].q, chosen);

		if(this->dModel->debugMode)
		{
			std::cout << "connecting [";
			this->dModel->printQLine(*tree[nearest.first].q, false, false);
			std::cout << "] towards [";
			this->dModel->printQLine(chosen, false, false);
			std::cout << "] using {" << edgeAction << "}" << std::endl;
		}

		Vertex lastAddedVertex = nearest.first;

		bool reached = false;

		if(distance <= this->delta)
			reached = true;

		::rl::plan::VectorPtr last = ::std::make_shared< ::rl::math::Vector >(this->dModel->getDof());

		//std::cout << "step: " << step << std::endl;
		//std::cout << "distance: " << distance << std::endl;

		if(!reached)
			this->dModel->interpolate(*tree[nearest.first].q, chosen, step / distance, *last, edgeAction);
		else
			*last = chosen;

		//std::cout << "last is in deg: ";
		//this->dModel->printQLine(*last, false, true, false, true);

		this->dModel->setPosition(*last);
		this->dModel->updateFrames();
		if(this->dModel->isColliding(edgeAction))
			return false;

		// TODO: fix this bug, why do I have to do it twice ... ?
		this->dModel->setPosition(*last);
		this->dModel->updateFrames();
		if(this->dModel->isColliding(edgeAction))
		{
			//std::cout << "WTF?! last collides now: " << last->transpose() << ", " << edgeAction << std::endl;
			return false;
		}

		::rl::math::Vector next(this->dModel->getDof());

		::std::size_t stepsUntilExtend = (::std::size_t)(this->extendStep / step);
		::std::size_t currStepCount = 1;

		while (!reached)
		{
			distance = this->dModel->robotMovementDistance(*last, chosen);

			if(distance <= this->delta)
			{
				reached = true;
				*last = chosen;
				break;
			}

//				std::cout << "connecting(inner) [";
//				this->dModel->printQLine(*last, false, false);
//				std::cout << "] towards [";
//				this->dModel->printQLine(chosen, false, false);
//				std::cout << "]" << std::endl;
//
//				std::cout << "step: " << step << std::endl;
//				std::cout << "distance: " << distance << std::endl;

			this->dModel->interpolate(*last, chosen, step / distance, next, edgeAction);

//				std::cout << "*last" << last->transpose() << std::endl;
//				std::cout << "chosen: " << chosen.transpose() << std::endl;
//				std::cout << "next: " << next.transpose() << std::endl;

			rl::math::Real robot_distance_joint = (last->segment(0, this->dModel->getDof(0)) - next.segment(0, this->dModel->getDof(0))).norm();
			if(robot_distance_joint < 1e-5)
			{
				// robot and object motion do not converge
				break;
			}

			this->dModel->setPosition(next);
			this->dModel->updateFrames();

			if(this->dModel->isColliding(edgeAction))
			{
				//std::cout << "collides!" << std::endl;
				//HERE IT LEAVES my strange bug ...
				break;
			}

			last = ::std::make_shared< ::rl::math::Vector >(this->dModel->getDof());
			*last = next;

			currStepCount++;
			if(currStepCount >= stepsUntilExtend)
			{
				if(this->dModel->debugMode)
				{
					std::cout << "Add vertex: ";
					this->dModel->printQLine(*last, false, false);
					std::cout << " (during connect after " << currStepCount << " steps)" << std::endl;
				}

				// TODO: delete me if no longer waiting for ENTER-Key needed
				//while(std::cin.get()!='\n');

				// TODO: only add this new vertex/edge if it isn't already contains (e.g. for transit to object, the object-vertex will be added each time for now
				/*for(VertexIteratorPair i = ::boost::vertices(this->tree[0]); i.first != i.second; ++i.first)
				{
					::rl::math::Vector qt1 = *this->tree[0][*i.first].q;
					::rl::math::Vector qt2 = *last.get();
					bool isEqual = true;
					for(::std::size_t i = 0; i<qt1.size(); ++i)
					{
						if(qt1(i) != qt2(i))
						{
							isEqual = false;
							break;
						}
					}
					if(isEqual)
					{
						this->dModel->printQLine(qt2, false, false);
						std::cout << " - with - ";
						this->dModel->printQLine(qt1, false, true);
						this->countEqualVec++;
					}
				}
				std::cout << "++++++++++ countEqualVec: " << this->countEqualVec << std::endl;*/

				vertex = this->addVertex(tree, last);
				this->addEdge(lastAddedVertex, vertex, edgeAction, tree);
				lastAddedVertex = vertex;

				currStepCount = 0;
			}
		}

		//if(reached)
		//if(currStepCount > 0)
		if((reached && currStepCount > 0) || currStepCount > 1)
		{
			/*this->dModel->setPosition(*last);
			this->dModel->updateFrames();
			if(this->dModel->isColliding(edgeAction))
			{
				std::cout << "last collides (5): " << last->transpose() << ", " << edgeAction << std::endl;
				std::cout << "SIMSALABIM " << currStepCount << std::endl;
				for(unsigned int i=0; i<30; i++)
				{
					this->dModel->setPosition(*last);
					this->dModel->updateFrames();
					if(this->dModel->isColliding(edgeAction))
						std::cout << "#";
					else
						std::cout << "+";

					this->dModel->setPosition(next);
					this->dModel->updateFrames();
					if(this->dModel->isColliding(edgeAction));
				}
				std::cout << std::endl;
			}*/

			if(this->dModel->debugMode)
			{
				std::cout << "Add vertex: ";
				this->dModel->printQLine(*last, false, false);
				if(!reached)
					std::cout << " (truncated)";
				std::cout << std::endl;
			}

			/*std::cout << "(Connecting ";
			this->dModel->printQLine(*last, false, false);
			std::cout << " with ";
			this->dModel->printQLine(*tree[lastAddedVertex].q, false, false);
			std::cout << ")" << std::endl;*/


			// TODO: delete me if no longer waiting for ENTER-Key needed
			//while(std::cin.get()!='\n');

			// TODO: only add this new vertex/edge if it isn't already contained (e.g. for transit to object, the object-vertex will be added each time for now)

			/*for(VertexIteratorPair i = ::boost::vertices(this->tree[0]); i.first != i.second; ++i.first)
			{
				::rl::math::Vector qt1 = *this->tree[0][*i.first].q;
				::rl::math::Vector qt2 = *last.get();
				bool isEqual = true;
				for(::std::size_t i = 0; i<qt1.size(); ++i)
				{
					if(qt1(i) != qt2(i))
					{
						isEqual = false;
						break;
					}
				}
				if(isEqual)
				{
					this->dModel->printQLine(qt2, false, false);
					std::cout << " - with - ";
					this->dModel->printQLine(qt1, false, true);
					this->countEqualVec++;
				}
			}
			std::cout << "++++++++++ countEqualVec: " << this->countEqualVec << std::endl;*/

			vertex = this->addVertex(tree, last);
			this->addEdge(lastAddedVertex, vertex, edgeAction, tree);
		}

		if(reached)
			return true;
		else
			return false;
	}

	// TODO: synchronize with isCollisionFree
	bool DamaRrt::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen, ::std::string edgeAction, Vertex& vertex)
	{
		if(this->dModel->debugMode)
		{
			std::cout << "connecting [";
			this->dModel->printQLine(*tree[nearest.first].q, false, false);
			std::cout << "] towards [";
			this->dModel->printQLine(chosen, false, false);
			std::cout << "] using {" << edgeAction << "}" << std::endl;
		}

		Vertex lastAddedVertex = nearest.first;
		::rl::plan::VectorPtr last = ::std::make_shared< ::rl::math::Vector >(this->dModel->getDof());
		*last = *tree[nearest.first].q;
		::rl::math::Vector next(this->dModel->getDof());
		::std::size_t stepsUntilExtend = (::std::size_t)(this->extendStep / this->delta);
		::std::size_t currStepCount = 1;
		bool reached = false;

		::rl::math::Real distance = this->dModel->metricConnectInterpolate(*last, chosen);

		if(distance <= this->delta)
			reached = true;

		if(!reached)
			this->dModel->interpolate(*tree[nearest.first].q, chosen, this->delta / distance, *last, edgeAction);
		else
			*last = chosen;

		//std::cout << "last is in deg: ";
		//this->dModel->printQLine(*last, false, true, false, true);

		this->dModel->setPosition(*last);
		this->dModel->updateFrames();
		if(this->dModel->isColliding(edgeAction))
			return false;

		// TODO: fix this bug, why do I have to do it twice ... ?
		this->dModel->setPosition(*last);
		this->dModel->updateFrames();
		if(this->dModel->isColliding(edgeAction))
			return false;

		while (!reached)
		{
			distance = this->dModel->metricConnectInterpolate(*last, chosen);

			if(distance <= this->delta)
			{
				reached = true;
				*last = chosen;
				break;
			}

			//std::cout << "delta: " << this->delta << std::endl;
			//std::cout << "distance: " << distance << std::endl;

			this->dModel->interpolate(*last, chosen, this->delta / distance, next, edgeAction);

			//std::cout << "*tree[nearest.first].q" << (tree[nearest.first].q)->transpose() << std::endl;
			//std::cout << "*last" << last->transpose() << std::endl;
			//std::cout << "chosen: " << chosen.transpose() << std::endl;
			//std::cout << "next: " << next.transpose() << std::endl;

			/*if((last->segment(0, this->dModel->getDof(0)) - next.segment(0, this->dModel->getDof(0))).norm() < 1e-5)
			{
				std::cerr << "Something is wrong in DamaRrt::connect(): robot and object motion do not converge! Fix me!" << std::endl;
				//break;
			}*/

			this->dModel->setPosition(next);
			this->dModel->updateFrames();

			if(this->dModel->isColliding(edgeAction))
			{
				break;
			}

			last = ::std::make_shared< ::rl::math::Vector >(this->dModel->getDof());
			*last = next;

			currStepCount++;
			if(currStepCount >= stepsUntilExtend)
			{
				if(this->dModel->debugMode)
				{
					std::cout << "Add vertex: ";
					this->dModel->printQLine(*last, false, false);
					std::cout << " (during connect after " << currStepCount << " steps)" << std::endl;
				}

				vertex = this->addVertex(tree, last);
				this->addEdge(lastAddedVertex, vertex, edgeAction, tree);
				lastAddedVertex = vertex;

				currStepCount = 0;
			}
		}

		if((reached && currStepCount > 0) || currStepCount > 1)
		{
			if(this->dModel->debugMode)
			{
				std::cout << "Add vertex: ";
				this->dModel->printQLine(*last, false, false);
				if(!reached)
					std::cout << " (truncated)";
				std::cout << std::endl;
			}

			vertex = this->addVertex(tree, last);
			this->addEdge(lastAddedVertex, vertex, edgeAction, tree);
		}

		if(reached)
			return true;
		else
			return false;
	}

	// TODO: synchronize with connect
	bool DamaRrt::isCollisionFree(const ::rl::math::Vector& v1, const ::rl::math::Vector& v2, ::std::string edgeAction, ::rl::plan::VectorList& pathSegment, ::std::deque< ::std::string >& pathSegmentAction)
	{
		if(this->dModel->debugMode)
		{
			std::cout << "test collision from [";
			this->dModel->printQLine(v1, false, false);
			std::cout << "] towards [";
			this->dModel->printQLine(v2, false, false);
			std::cout << "] using {" << edgeAction << "}" << std::endl;
		}

		pathSegment.push_back(v1);
		pathSegmentAction.push_back(edgeAction);

		::rl::plan::VectorPtr last = ::std::make_shared< ::rl::math::Vector >(this->dModel->getDof());
		*last = v1;
		::rl::math::Vector next(this->dModel->getDof());
		::std::size_t stepsUntilExtend = (::std::size_t)(this->extendStep / this->delta);
		::std::size_t currStepCount = 1;
		bool reached = false;

		::rl::math::Real distance = this->dModel->metricConnectInterpolate(*last, v2);

		if(distance <= this->delta)
			reached = true;

		if(!reached)
			this->dModel->interpolate(v1, v2, this->delta / distance, *last, edgeAction);
		else
			*last = v2;

		//std::cout << "last is in deg: ";
		//this->dModel->printQLine(*last, false, true, false, true);

		this->dModel->setPosition(*last);
		this->dModel->updateFrames();
		if(this->dModel->isColliding(edgeAction))
			return false;

		// TODO: fix this bug, why do I have to do it twice ... ?
		this->dModel->setPosition(*last);
		this->dModel->updateFrames();
		if(this->dModel->isColliding(edgeAction))
			return false;

		while (!reached)
		{
			distance = this->dModel->metricConnectInterpolate(*last, v2);

			if(distance <= this->delta)
			{
				reached = true;
				*last = v2;
				break;
			}

			//std::cout << "delta: " << this->delta << std::endl;
			//std::cout << "distance: " << distance << std::endl;

			this->dModel->interpolate(*last, v2, this->delta / distance, next, edgeAction);

			//std::cout << "*tree[nearest.first].q" << (tree[nearest.first].q)->transpose() << std::endl;
			//std::cout << "*last" << last->transpose() << std::endl;
			//std::cout << "chosen: " << chosen.transpose() << std::endl;
			//std::cout << "next: " << next.transpose() << std::endl;

			/*if((last->segment(0, this->dModel->getDof(0)) - next.segment(0, this->dModel->getDof(0))).norm() < 1e-5)
			{
				std::cerr << "Something is wrong in DamaRrt::connect(): robot and object motion do not converge! Fix me!" << std::endl;
				//break;
			}*/

			this->dModel->setPosition(next);
			this->dModel->updateFrames();

			if(this->dModel->isColliding(edgeAction))
			{
				break;
			}

			last = ::std::make_shared< ::rl::math::Vector >(this->dModel->getDof());
			*last = next;

			currStepCount++;
			if(currStepCount >= stepsUntilExtend)
			{
				pathSegment.push_back(*last);
				pathSegmentAction.push_back(edgeAction);
				currStepCount = 0;
			}
		}

		if((reached && currStepCount > 0) || currStepCount > 1)
		{
			pathSegment.push_back(*last);
			pathSegmentAction.push_back(edgeAction);
		}

		if(reached)
			return true;
		else
			return false;
	}

	DamaRrtAction::Edge DamaRrt::addEdge(const Vertex& u, const Vertex& v, ::std::string edgeAction, Tree& tree, bool drawIfPossible)
	{
		Edge e = ::boost::add_edge(u, v, tree).first;
		tree[e].action = edgeAction;

		if (NULL != this->viewer && drawIfPossible)
		{
			this->viewer->drawConfigurationEdge(*tree[u].q, *tree[v].q);
		}

		//++this->edgeCount;

		return e;
	}

	DamaRrtAction::Vertex DamaRrt::addVertex(Tree& tree, const ::rl::plan::VectorPtr& q)
	{
		Vertex v = ::boost::add_vertex(tree);
		tree[v].index = ::boost::num_vertices(tree) - 1;
		tree[v].q = q;
		tree[v].radius = ::std::numeric_limits< ::rl::math::Real >::max();

		if (this->kd)
		{
			this->addPoint(tree[::boost::graph_bundle].nn, QueryItem(q.get(), v));
		}

		if (NULL != this->viewer)
		{
			//this->viewer->drawConfigurationVertex(*tree[v].q);
		}

		//++this->vertexCount;

		return v;
	}

	void DamaRrt::getPath(::rl::plan::VectorList& path, ::std::deque< ::std::string >& actions)
	{
		Vertex i = this->end[0];

		while (::boost::in_degree(i, this->tree[0]) > 0)
		{
			Edge e = *::boost::in_edges(i, this->tree[0]).first;
			path.push_front(*this->tree[0][i].q);
			actions.push_front(this->tree[0][e].action);
			//this->dModel->printQLine(path.front(), false, true);
			i = ::boost::source(*::boost::in_edges(i, this->tree[0]).first, this->tree[0]);
		}
		//this->dModel->printQLine(path.front(), false, true);

		path.push_front(*this->tree[0][i].q);

		/*::std::cout << "--> last tree 1 node: ";
		this->dModel->printQLine(path.back(), false, true);
		::std::cout << "--> nodes: " << path.size() << ::std::endl;
		this->dModel->printQLine(path.front(), false, true);
		::std::cout << ::std::endl << ::std::endl;*/

		if(this->twoTreeVersion)
		{
			i = this->end[1];
			while (::boost::in_degree(i, this->tree[1]) > 0)
			{
				Edge e = *::boost::in_edges(i, this->tree[1]).first;
				i = ::boost::source(e, this->tree[1]);
				path.push_back(*this->tree[1][i].q);
				actions.push_back(this->tree[1][e].action);
			}
		}
	}

	void DamaRrt::getObjectPath(::rl::plan::VectorList& path)
	{
		rl::plan::VectorList fullPath;
		::std::deque< ::std::string > actions;
		this->getPath(fullPath, actions);

		rl::plan::VectorList::iterator i = fullPath.begin();
		rl::plan::VectorList::iterator j = ++fullPath.begin();
		::std::deque< ::std::string >::iterator e = actions.begin();
		bool currManipulating = false;
		::std::size_t lastObjectNr = 0;
		::std::size_t currObjectNr = 0;
		path.push_back(*i);
		for(; i != fullPath.end() && j != fullPath.end() && e != actions.end(); ++i, ++j, ++e)
		{
			if(!currManipulating && *e != DamaPrimTransit::getInstance()->getName())
			{
				currManipulating = true;
				currObjectNr = boost::lexical_cast< ::std::size_t >(e->substr(e->find_last_of(" ")+1));
				if(currObjectNr != lastObjectNr && lastObjectNr != 0)	// latter statement prevents initial push_back
					path.push_back(*i);
				lastObjectNr = currObjectNr;
			}
			else if(currManipulating && *e == DamaPrimTransit::getInstance()->getName())
			{
				currManipulating = false;
			}
		}
		path.push_back(*i);
	}

	void DamaRrt::writeGraphML(Tree& tree, ::std::string fileName)
	{
		if(this->dModel->debugMode)
			std::cout << "### Writing GraphML File '" << fileName << "' ...";

		// define your Vertex Index Map
		typedef Tree::vertex_descriptor NodeID;
		typedef std::map<NodeID, std::size_t> IndexMap;
		IndexMap mapIndex;
		boost::associative_property_map<IndexMap> propmapIndex(mapIndex);

		int i = 0;
		BGL_FORALL_VERTICES(v, tree, Tree)
		{
			put(propmapIndex, v, i++);
		}

		boost::dynamic_properties dp;
		dp.property("index", get(&VertexBundle::index, tree));
		dp.property("q", get(&VertexBundle::q, tree));
		dp.property("radius", get(&VertexBundle::radius, tree));
		dp.property("action", get(&EdgeBundle::action, tree));
		//dp.property("t", get(&VertexBundle::t, this->tree[0]));

		std::ofstream myfile;
		myfile.open(fileName.c_str());
		write_graphml(myfile, tree, propmapIndex, dp, true);
		myfile.close();

		if(this->dModel->debugMode)
		{
			std::cout << " done. ###" << std::endl;
			std::cout << std::endl << std::endl;
		}
	}

	void DamaRrt::finalizeWork(Vertex* end0, Vertex* end1)
	{
		if(!this->twoTreeVersion)
		{
			this->end[0] = *end0;
			this->writeGraphML(this->tree[0], tree0FileName);

			if(this->dModel->debugMode)
				std::cout << "### Writing GraphML Meta File '" << metaFileName << "' ...";
			std::ofstream myfile;
			myfile.open(metaFileName.c_str());
			myfile << this->twoTreeVersion << std::endl;
			myfile << this->tree[0][*end0].index << std::endl;
			myfile.close();
			if(this->dModel->debugMode)
			{
				std::cout << " done. ###" << std::endl;
				std::cout << std::endl << std::endl;
			}
		}
		else
		{
			this->end[0] = *end0;
			this->end[1] = *end1;
			this->writeGraphML(this->tree[0], tree0FileName);
			this->writeGraphML(this->tree[1], tree1FileName);

			if(this->dModel->debugMode)
				std::cout << "### Writing GraphML Meta File '" << metaFileName << "' ...";
			std::ofstream myfile;
			myfile.open(metaFileName.c_str());
			myfile << this->twoTreeVersion << std::endl;
			myfile << this->tree[0][*end0].index << std::endl;
			myfile << this->tree[1][*end1].index << std::endl;
			myfile.close();
			if(this->dModel->debugMode)
			{
				std::cout << " done. ###" << std::endl;
				std::cout << std::endl << std::endl;
			}
		}

		/*std::cout << "end0 {" << this->tree[0][*end0].index << "}: ";
		this->dModel->printQLine(*(this->tree[0][*end0].q.get()), false, true);
		if(this->twoTreeVersion)
		{
			std::cout << "end1 {" << this->tree[1][*end1].index << "}: ";
			this->dModel->printQLine(*(this->tree[1][*end1].q.get()), false, true);
		}

		std::cout << "Vertex list (tree 0):" << std::endl;
		for(VertexIteratorPair i = ::boost::vertices(this->tree[0]); i.first != i.second; ++i.first)
		{
			std::cout << "tree 0: " << this->tree[0][*i.first].index << ": ";
			this->dModel->printQLine(*this->tree[0][*i.first].q);
		}

		std::cout << "Edge list (tree 0):" << std::endl;
		for(EdgeIteratorPair i = ::boost::edges(this->tree[0]); i.first != i.second; ++i.first)
		{
			Edge e = *i.first;
			Vertex u = ::boost::source(e, this->tree[0]);
			Vertex v = ::boost::target(e, this->tree[0]);
			std::cout << "tree 0: [";
			this->dModel->printQLine(*this->tree[0][u].q, false, false);
			std::cout << " {" << this->tree[0][u].index << "}] to [";
			this->dModel->printQLine(*this->tree[0][v].q, false, false);
			std::cout << " {" << this->tree[0][v].index << "}]" << std::endl;
		}

		if(this->twoTreeVersion)
		{
			std::cout << "Vertex list (tree 1):" << std::endl;
			for(VertexIteratorPair i = ::boost::vertices(this->tree[1]); i.first != i.second; ++i.first)
			{
				std::cout << "tree 1: " << this->tree[1][*i.first].index << ": ";
				this->dModel->printQLine(*this->tree[1][*i.first].q);
			}

			std::cout << "Edge list (tree 1):" << std::endl;
			for(EdgeIteratorPair i = ::boost::edges(this->tree[1]); i.first != i.second; ++i.first)
			{
				Edge e = *i.first;
				Vertex u = ::boost::source(e, this->tree[1]);
				Vertex v = ::boost::target(e, this->tree[1]);
				std::cout << "tree 1: [";
				this->dModel->printQLine(*this->tree[1][u].q, false, false);
				std::cout << " {" << this->tree[1][u].index << "}] to [";
				this->dModel->printQLine(*this->tree[1][v].q, false, false);
				std::cout << " {" << this->tree[1][v].index << "}]" << std::endl;
			}
		}*/
	}

	bool DamaRrt::postProc(::rl::plan::VectorList& path, ::std::deque< ::std::string >& actions)
	{
		if(this->dModel->pathSmoothing)
			pathSmoothing(path, actions);

		// Extend the pushes by manipulating the final path/actions manually
		bool extendPushesSuccess = false;
		extendPushesSuccess = extendPushes(path, actions);

		// Calculate the resulting trajectory, and overwrite path/actions
		//calcTrajectory(path, actions); <-- not in this project anymore

		// a) Print final robot path
		// b) Construct a tree for the final robot movement in order to visualize the solution path with graph-tools
		// c) Write vertices and edges of the solution path in separate files
		if(this->dModel->debugMode)
			::std::cout << "Final Robot Path:" << ::std::endl;
		Tree treeSolution;
		Vertex v1, v2;
		std::ofstream pathFileVertices(this->pathSolVerticesFileName.c_str());
		std::ofstream pathFileEdges(this->pathSolEdgesFileName.c_str());
		rl::plan::VectorList::iterator i = path.begin();
		rl::plan::VectorList::iterator j = ++path.begin();
		::std::deque< ::std::string >::iterator e = actions.begin();
		v1 = this->addVertex(treeSolution, ::std::make_shared< ::rl::math::Vector >(*i));
		if(this->dModel->debugMode)
			this->dModel->printQLine(*i);
		this->dModel->printQLine(*i, false, true, false, true, pathFileVertices);
		for(; i != path.end() && j != path.end() && e != actions.end(); ++i, ++j, ++e)
		{
			v2 = this->addVertex(treeSolution, ::std::make_shared< ::rl::math::Vector >(*j));
			this->addEdge(v1, v2, *e, treeSolution, false);

			if(this->dModel->debugMode)
			{
				::std::cout << *e << ::std::endl;
				this->dModel->printQLine(*j);
			}

			this->dModel->printQLine(*j, false, true, false, true, pathFileVertices);
			pathFileEdges << *e << ::std::endl;

			v1 = v2;
		}
		pathFileVertices.close();
		pathFileEdges.close();

		if(this->dModel->debugMode)
		{
			::std::cout << ::std::endl << "==> " << path.size() << " vertices and " << actions.size() << " edges" << ::std::endl << ::std::endl;
			::std::cout << "Extend Pushes: " << (extendPushesSuccess?"success":"fail") << ::std::endl << ::std::endl;
		}

		if(this->hierarchicalVersion)
			this->writeGraphML(treeSolution, tree0FileName);
		this->writeGraphML(treeSolution, treeSolFileName);

		if(this->hierarchicalVersion)
		{
			std::ofstream myfile;
			myfile.open(metaFileName.c_str());
			myfile << false << std::endl;
			myfile << treeSolution[v2].index << std::endl;
			myfile.close();
		}

		return true;
	}

	bool DamaRrt::extendPushes(::rl::plan::VectorList& path, ::std::deque< ::std::string >& actions)
	{
		bool extendedAll = true;

		if(this->dModel->extendPushes)
		{
			if(this->dModel->debugMode)
				::std::cout << "extendPushes:" << ::std::endl;

			rl::plan::VectorList::iterator i = path.begin();
			rl::plan::VectorList::iterator j = ++path.begin();
			::std::deque< ::std::string >::iterator e = actions.begin();
			::rl::math::Vector P1, P2;
			::std::string pushPrefix = "Push ";
			::std::string lastAction = DamaPrimTransit::getInstance()->getName();
			::std::string currTraversingPushEdge = "";
			for(; i != path.end() && j != path.end() && e != actions.end(); ++i, ++j, ++e)
			{
				if(*e != lastAction)
				{
					if(e->substr(0, pushPrefix.size()) == pushPrefix)
					{
						P1 = *i;
						currTraversingPushEdge = *e;
					}
					else if(currTraversingPushEdge != "")
					{
						P2 = *i;

						// extend here the push between p1 and p2
						::rl::math::Transform T1 = this->dModel->calcForwardKinematics(P1.segment(0, dModel->getDof(0)), 0);
						::rl::math::Transform T2 = this->dModel->calcForwardKinematics(P2.segment(0, dModel->getDof(0)), 0);

						::rl::math::Vector pushDir(2);
						pushDir(0) = T2.translation().x() - T1.translation().x();
						pushDir(1) = T2.translation().y() - T1.translation().y();
						pushDir.normalize();
						if(currTraversingPushEdge.substr(0, DamaPrimPushInterior::getInstance()->getName().size()) == DamaPrimPushInterior::getInstance()->getName())
							pushDir *= DamaPrimPushInterior::getInstance()->POSTPROC_EXTEND_DIST;
						else if(currTraversingPushEdge.substr(0, DamaPrimPushExterior::getInstance()->getName().size()) == DamaPrimPushExterior::getInstance()->getName())
							pushDir *= DamaPrimPushExterior::getInstance()->POSTPROC_EXTEND_DIST;
						else if(currTraversingPushEdge.substr(0, DamaPrimPushFrontal::getInstance()->getName().size()) == DamaPrimPushFrontal::getInstance()->getName())
							pushDir *= DamaPrimPushFrontal::getInstance()->POSTPROC_EXTEND_DIST;
						else if(currTraversingPushEdge.substr(0, DamaPrimPushMobile::getInstance()->getName().size()) == DamaPrimPushMobile::getInstance()->getName())
							pushDir *= DamaPrimPushMobile::getInstance()->POSTPROC_EXTEND_DIST;

						::rl::math::Transform T3 = T2;
						T3.translation().x() += pushDir(0);
						T3.translation().y() += pushDir(1);

						if(this->dModel->debugMode)
						{
							::std::cout << "### Extend " << currTraversingPushEdge << " from <";
							this->dModel->printQLine(P1, false, false, false, true);
							::std::cout << "> to <";
							this->dModel->printQLine(P2, false, false, false, true);
							::std::cout << "> with pushDir (" << pushDir(0) << " | " << pushDir(1) << "): ";
						}

						rl::math::Matrix66 constraint_position_orient = rl::math::Matrix66::Zero();
						constraint_position_orient.diagonal() << 1, 1, 1, 1, 1, 1;
						rl::math::Vector q_ik_start = P2.segment(0, dModel->getDof(0));
						::rl::math::Vector P3(P2);
						bool calcInverse = DamaModel::calcInversePositionTaskPosture(this->dModel->mdl, T3, q_ik_start, constraint_position_orient, this->dModel->coupleJoint1And2);
						if(calcInverse)
						{
							this->dModel->updateRobotPosition(P3);

							// insert P3 into path and actions right before the iterator i/e is pointing to
							path.insert(i, P3);
							actions.insert(e, DamaPrimTransit::getInstance()->getName());

							if(this->dModel->debugMode)
							{
								::std::cout << "<";
								this->dModel->printQLine(P3, false, false, false, true);
								::std::cout << ">" << ::std::endl << ::std::endl;
							}
						}
						else
						{
							extendedAll = false;

							if(this->dModel->debugMode)
								::std::cout << "IK failed!" << ::std::endl << ::std::endl;
						}

						currTraversingPushEdge = "";
					}
				}
				lastAction = *e;
			}

			if(this->dModel->debugMode)
				::std::cout << "==> extendPushes: " << (extendedAll?"success":"fail") << ::std::endl << ::std::endl;
		}
		else
		{
			if(this->dModel->debugMode)
				::std::cout << "extendPushes: nothing to extend." << ::std::endl << ::std::endl;
		}

		return extendedAll;
	}

	bool DamaRrt::pathSmoothing(::rl::plan::VectorList& path, ::std::deque< ::std::string >& actions)
	{
		dama::Timer smoothTimer;
		smoothTimer.start();

		rl::plan::VectorList::iterator t1;
		rl::plan::VectorList::iterator t2;
		::std::deque< ::std::string >::iterator te;

		/*t1 = path.begin();
		this->dModel->printQLine(*t1);
		for(t2 = ++path.begin(), te = actions.begin(); t1 != path.end() && t2 != path.end() && te != actions.end(); ++t1, ++t2, ++te)
		{
			::std::cout << *te << ::std::endl;
			this->dModel->printQLine(*t2);
		}*/

		::std::string currPrimitive = actions.front();
		::std::size_t startIndex = 0;
		::std::size_t endIndex = 0;

		// Smooth each primitive separately
		while(true)
		{
			// Determine endIndex for current primitive
			endIndex = path.size()-1;
			for(::std::size_t i=startIndex; i<actions.size(); ++i)
			{
				if(actions.at(i) != currPrimitive)
				{
					endIndex = i;
					break;
				}
			}

			// TODO: extend to other/all primitives ?!
			if(true/*currPrimitive == DamaPrimTransit::getInstance()->getName()*/)
			{
				if(this->dModel->debugMode)
					::std::cout << "# Start smoothing " << currPrimitive << " - from " << startIndex << " to " << endIndex << ::std::endl;
				// N iterations
				// TODO: iteration count should depend on the segment length etc.?!
				for(::std::size_t i=0; i<100; ++i)
				{
					// 1) Pick two vertices at random, v1 will be different from and smaller than v2
					::std::size_t v1index = ::std::rand() % (endIndex - startIndex + 1) + startIndex;
					::std::size_t v2index = ::std::rand() % (endIndex - startIndex + 1) + startIndex;
					while(v1index == v2index)
						v2index = ::std::rand() % (endIndex - startIndex + 1) + startIndex;
					if(v2index < v1index)
						std::swap(v1index,v2index);

					// 2) Check if there can be a shortcut at all AND if this shortcut is collision free
					if(v2index - v1index > 1)
					{
						rl::plan::VectorList::iterator v1 = path.begin();
						::std::advance(v1, v1index);
						rl::plan::VectorList::iterator v2 = path.begin();
						::std::advance(v2, v2index);
						::rl::plan::VectorList pathSegment;
						::std::deque< ::std::string > pathSegmentAction;
						if(this->isCollisionFree(*v1, *v2, currPrimitive, pathSegment, pathSegmentAction))
						{
							// TODO: is the shortcut really shorter?! check length or time ...

							// 3) Replace the original segment by the shortcut -> i.e., erase the intermediate segment
							//::std::cout << "Shortcut found - from " << v1index << " to " << v2index << ::std::endl;

							/*::std::cout << "# Shortcut: " << ::std::endl;
							t1 = pathSegment.begin();
							this->dModel->printQLine(*t1);
							for(t2 = ++pathSegment.begin(), te = pathSegmentAction.begin(); t1 != pathSegment.end() && t2 != pathSegment.end() && te != pathSegmentAction.end(); ++t1, ++t2, ++te)
							{
								::std::cout << *te << ::std::endl;
								this->dModel->printQLine(*t2);
							}*/

							/*::std::cout << "# Path before: " << ::std::endl;
							t1 = path.begin();
							this->dModel->printQLine(*t1);
							for(t2 = ++path.begin(), te = actions.begin(); t1 != path.end() && t2 != path.end() && te != actions.end(); ++t1, ++t2, ++te)
							{
								::std::cout << *te << ::std::endl;
								this->dModel->printQLine(*t2);
							}*/

							rl::plan::VectorList::iterator v1next = path.begin();
							::std::advance(v1next, v1index+1);
							path.erase(v1next, v2);
							rl::plan::VectorList::iterator pathSegmentInnerStart = ++pathSegment.begin();
							rl::plan::VectorList::iterator pathSegmentInnerEnd = --pathSegment.end();
							v1next = path.begin();
							::std::advance(v1next, v1index+1);
							path.insert(v1next, pathSegmentInnerStart, pathSegmentInnerEnd);
							actions.erase(actions.begin()+v1index+1, actions.begin()+v2index);
							actions.insert(actions.begin()+v1index+1, pathSegmentAction.begin()+1, pathSegmentAction.end()-1);

							/*::std::cout << "# Path after: " << ::std::endl;
							t1 = path.begin();
							this->dModel->printQLine(*t1);
							for(t2 = ++path.begin(), te = actions.begin(); t1 != path.end() && t2 != path.end() && te != actions.end(); ++t1, ++t2, ++te)
							{
								::std::cout << *te << ::std::endl;
								this->dModel->printQLine(*t2);
							}*/

							//::std::cout << "endIndex before: " << endIndex << ::std::endl;
							endIndex = endIndex - (v2index - v1index - 1) + (pathSegment.size() - 2);
							//::std::cout << "endIndex after: " << endIndex << ::std::endl;
						}
					}
				}
			}

			// Proceed with next primitive, if available
			if(endIndex != path.size()-1)
			{
				currPrimitive = actions.at(endIndex);
				startIndex = endIndex;
			}
			else
				break;
		}

		smoothTimer.stop();
		if(this->dModel->debugMode)
			::std::cout << "Path Smoothing took " << smoothTimer.elapsed() << " seconds." << ::std::endl;

		return true;
	}

	/** Note: depends on this->dModel->currIsChosen !!! */
	void DamaRrt::completePartialGoal(const ::rl::math::Vector& samplePart, ::rl::math::Vector& goalFull)
	{
		::std::vector<bool> goalDimDefinedTemp(*goalDimDefined);

		for(::std::size_t i=0; i<this->dModel->getDof(); ++i)
		{
			if(!this->goalDimDefined->at(i))
			{
				assert((*this->goal)(i) == 0.0);

				if(!this->dModel->currIsChosen.at(i))
					assert(samplePart(i) == 0.0);
				else
				{
					goalFull(i) = samplePart(i);
					goalDimDefinedTemp.at(i) = true;
				}
			}
			else
				goalFull(i) = (*this->goal)(i);
		}

		::std::size_t currIndex = 0;
		bool randomizeCurrSubspace = false;
		for(::std::size_t i=0; i<this->dModel->getNumMovableComponents(); ++i)
		{
			randomizeCurrSubspace = false;
			for(::std::size_t u=0; u<this->dModel->getDof(i); ++u)
			{
				if(!goalDimDefinedTemp.at(currIndex+u) && !this->dModel->currIsChosen.at(currIndex+u))
				{
					randomizeCurrSubspace = true;
					break;
				}
			}
			if(randomizeCurrSubspace)
				this->dSampler->generateOnlyCollisionFree(goalFull, i, this->randSupportSurface(), &goalDimDefinedTemp);

			currIndex += this->dModel->getDof(i);
		}
	}

	void DamaRrt::sampleFromGoalFull(::rl::math::Vector &sample)
	{
		::std::vector<bool> goalDimDefinedTemp(*goalDimDefined);

		::std::size_t currIndex = 0;
		bool fillCurrSubspace = false;
		for(::std::size_t i=0; i<this->dModel->getNumMovableComponents(); ++i)
		{
			fillCurrSubspace = false;
			for(::std::size_t u=0; u<this->dModel->getDof(i); ++u)
			{
				if(!goalDimDefinedTemp.at(currIndex+u))
				{
					fillCurrSubspace = true;
					break;
				}
			}
			if(fillCurrSubspace)
			{
				if(this->rand() > 0.2f)
					this->dSampler->generateOnlyCollisionFree(sample, i, this->randSupportSurface(), &goalDimDefinedTemp);
				else
				{
					for(::std::size_t u=0; u<this->dModel->getDof(i); ++u)
						if(!goalDimDefinedTemp.at(currIndex+u))
							sample(currIndex+u) = (*this->start)(currIndex+u);
				}
			}

			currIndex += this->dModel->getDof(i);
		}
	}

	void DamaRrt::setConfig(const bool twoTreeVersion, const bool hierarchicalVersion, const bool useAccurateDistance)
	{
		this->reset();

		this->twoTreeVersion = twoTreeVersion;
		if(this->twoTreeVersion)
		{
			begin.resize(2, NULL);
			end.resize(2, NULL);
			tree.resize(2);
		}
		else
		{
			begin.resize(1, NULL);
			end.resize(1, NULL);
			tree.resize(1);
		}

		this->hierarchicalVersion = hierarchicalVersion;

		this->useAccurateDistance = useAccurateDistance;
	}

	void DamaRrt::resetStatistics()
	{
		if(this->dModel != NULL)
		{
			this->dModel->reset();
			this->dModel->timeIK = 0.0;
		}
		this->iterationCount = 0;
		this->edgeCount = 0;
		this->vertexCount = 0;
		this->timeSampling = 0;
		this->timeNNSearch = 0;
		this->timePropagate = 0;
		this->timeConnect = 0;
	}
}
