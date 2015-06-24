/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#include <boost/lexical_cast.hpp>
#include <rl/sg/Body.h>
#include <rl/sg/SimpleScene.h>

#include "DamaModel.h"

#include <fstream>

namespace dama
{
	rl::math::Real DamaModel::ikPostureGainMaxStep = 20.0 * rl::math::DEG2RAD;
	bool DamaModel::ikPrematureQuit = false;

	DamaModel::DamaModel() :
		DistanceModel()
	{
		// TODO: currently no wraparound implemented -> affects (only?) DamaModel::interpolate and DamaModel::minDistanceToRectangle

		// temporary variables; will change during planning
		this->timeIK = 0.0;
		this->forwardSearch = true;
		this->checkCollisionRobotObjects = true;
	}

	DamaModel::~DamaModel()
	{
	}

	void DamaModel::printQLine(const ::rl::math::Vector& q, bool showDefinedState, bool addNewLine, bool printFullState, bool jointsInDeg, ::std::ostream& os) const
	{
		::std::size_t robotDoF = 0;
		if(NULL != this->kin)
			robotDoF = this->kin->getDof();
		else
			robotDoF = this->mdl->getDof();

		for(::std::size_t i=0; i<q.size(); i++)
		{
			if(i < robotDoF && jointsInDeg && this->allRevoluteJoints)
				os << (q(i) * rl::math::RAD2DEG);
			else
				os << q(i);

			if(showDefinedState)
				os << " (" << this->currIsChosen.at(i) << ")";

			if(i < q.size()-1)
			{
				if(i < robotDoF && printFullState)
					os << " | ";
				else
					os << ", ";
			}
		}
		if(addNewLine)
			os << std::endl;
	}

	void DamaModel::printTransform(::rl::math::Transform& t, bool addNewLine) const
	{
		rl::math::Transform::TranslationPart position = t.translation();
		rl::math::Vector3 orientation = t.rotation().eulerAngles(2, 1, 0).reverse();

		::std::cout << position.x() << " | " << position.y() << " | " << position.z() << " || " << (orientation.x() * rl::math::RAD2DEG) << " | " << (orientation.y() * rl::math::RAD2DEG) << " | " << (orientation.z() * rl::math::RAD2DEG);

		if(addNewLine)
			std::cout << std::endl;
	}

	::std::size_t DamaModel::getDof() const
	{
		::std::size_t sumDoF = 0;
		if (NULL != this->kin)
		{
			sumDoF += this->numRobots * this->kin->getDof();
		}
		else
		{
			sumDoF += this->numRobots * this->mdl->getDof();
		}
		sumDoF += this->numObjects * this->dimObjects;
		return sumDoF;
	}

	::std::size_t DamaModel::getDof(::std::size_t index) const
	{
		if(index < this->numRobots)
		{
			if (NULL != this->kin)
			{
				return this->kin->getDof();
			}
			else
			{
				return this->mdl->getDof();
			}
		}
		else
			return this->dimObjects;
	}

	void DamaModel::getMaximum(::rl::math::Vector& maximum) const
	{
		std::cerr << "getMaximum() should not be used within DamaModel without specifying a subspace" << std::endl;
	}

	void DamaModel::getMaximum(::rl::math::Vector& maximum, ::std::size_t subspace) const
	{
		if(subspace < this->numRobots)
		{
			if (NULL != this->kin)
			{
				this->kin->getMaximum(maximum);
			}
			else
			{
				this->mdl->getMaximum(maximum);
			}
			::rl::math::Vector min(this->getDof(subspace));
			::rl::math::Vector max(this->getDof(subspace));
			this->mdl->getMinimum(min);
			this->mdl->getMaximum(max);
			::rl::math::Vector range = max - min;
			maximum.segment(0, this->getDof(subspace)) -= (1.0 - this->kinematicSoftRangeScale) * range / 2;
			//::std::cout << "Final Maximum:" << ::std::endl;
			//::std::cout << maximum << ::std::endl;
		}
		else
		{
			for(::std::size_t i=0; i<this->dimObjects; i++)
			{
				maximum(i) = this->maximumObjects(i);
			}
		}
	}

	void DamaModel::getMinimum(::rl::math::Vector& minimum) const
	{
		std::cerr << "getMinimum() should not be used within DamaModel without specifying a subspace" << std::endl;
	}

	void DamaModel::getMinimum(::rl::math::Vector& minimum, ::std::size_t subspace) const
	{
		if(subspace < this->numRobots)
		{
			if (NULL != this->kin)
			{
				this->kin->getMinimum(minimum);
			}
			else
			{
				this->mdl->getMinimum(minimum);
			}
			::rl::math::Vector min(this->getDof(subspace));
			::rl::math::Vector max(this->getDof(subspace));
			this->mdl->getMinimum(min);
			this->mdl->getMaximum(max);
			::rl::math::Vector range = max - min;
			minimum.segment(0, this->getDof(subspace)) += (1.0 - this->kinematicSoftRangeScale) * range / 2;
			//::std::cout << "Final Minimum:" << ::std::endl;
			//::std::cout << minimum << ::std::endl;
		}
		else
		{
			for(::std::size_t i=0; i<this->dimObjects; i++)
			{
				minimum(i) = this->minimumObjects(i);
			}
		}
	}

	bool DamaModel::isValid(const ::rl::math::Vector& q) const
	{
		if (NULL != this->kin)
		{
			return this->kin->isValid(this->getSubspace(q, 0));
		}
		else
		{
			return this->mdl->isValid(this->getSubspace(q, 0));
		}

		// TODO: check also other subspaces
	}

	::rl::math::Vector DamaModel::getSubspace(const ::rl::math::Vector& q, ::std::size_t subspace) const
	{
		::std::size_t currIndex = this->indexFromSubspace(subspace);

		::rl::math::Vector res(this->getDof(subspace));

		for(::std::size_t i=0; i<this->getDof(subspace); ++i)
			res(i) = q(currIndex+i);

		return res;
	}

	::std::size_t DamaModel::getNumMovableComponents() const
	{
		return this->numRobots + this->numObjects;
	}

	::rl::math::Real DamaModel::distance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const
	{
		return this->inverseOfTransformedDistance(this->transformedDistance(q1, q2));
	}

	::rl::math::Real DamaModel::inverseOfTransformedDistance(const ::rl::math::Real& d) const
	{
		return ::std::sqrt(d);
	}

	::rl::math::Real DamaModel::transformedDistance(const ::rl::math::Real& d) const
	{
		return ::std::pow(d, 2);
	}

	::rl::math::Real DamaModel::transformedDistance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const
	{
		::rl::math::Real dist = 0;
		::rl::math::Vector q2Full(this->getDof());
		this->completePartialSample(q2, q1, q2Full);

		// return max distance if the completed full sample has two (or more) objects not on a support surface
		bool anObjectIsNotOnSS = false;
		::std::size_t objectSubspaceNotOnSS = 0;
		for(::std::size_t i=this->numRobots; i<this->getNumMovableComponents(); ++i)
		{
			if(!this->isOnSupportSurface(q2Full, i))
			{
				if(anObjectIsNotOnSS)
					return ::std::numeric_limits< ::rl::math::Real >::max();
				anObjectIsNotOnSS = true;
				objectSubspaceNotOnSS = i;
			}
		}
		// return max distance if the completed sample received an object not on a support surface but the robot pose was already set -> object won't be grasped!
		if(anObjectIsNotOnSS && !this->currIsChosen.at(this->indexFromSubspace(objectSubspaceNotOnSS)) && this->currIsChosen.at(0))
		{
			return ::std::numeric_limits< ::rl::math::Real >::max();
		}

		if(this->dRrt->getUseAccurateDistance())
		{
			::std::vector< ::rl::math::Vector > vecRes;
			::std::vector< ::std::string > vecEdgeAction;
			ESP::Res res = this->emptySpacePlanner(vecRes, q1, q2Full, vecEdgeAction, false);
			switch(res)
			{
			case ESP::OK:
				for(::std::size_t u=0; u<vecRes.size()-1; ++u)
					dist += robotMovementDistance(vecRes.at(u), vecRes.at(u+1));
				break;
			case ESP::UNREACHABLE:
				dist = ::std::numeric_limits< ::rl::math::Real >::max();
				break;
			case ESP::ALREADY_THERE:
				dist = 0;
				break;
			default:
				std::cerr << "DamaModel::transformedDistance: Unhandled case of ESP::Res res." << std::endl;
			}
			dist = this->transformedDistance(dist);
		}
		else
		{
			::rl::math::Real metMovingObjectPenalty;
			::rl::math::Real metMoveToObjectPenalty;
			::rl::math::Real metMoveToObjectFactor;

			if(this->metType == "MaxComponentMovement")
			{
				::rl::math::Real maxDist = 0;
				for(::std::size_t i=0; i<this->getNumMovableComponents(); ++i)
				{
					::rl::math::Real currSubspaceDist = this->metricSubspaceDistance(q1, q2Full, i);
					if(currSubspaceDist > maxDist)
						maxDist = currSubspaceDist;
				}
				dist = this->transformedDistance(maxDist);
			}
			else if(this->metType == "SumComponentMovementWithObjectPenalty")
			{
				for(::std::size_t i=0; i<this->getNumMovableComponents(); ++i)
				{
					::rl::math::Real currSubspaceDist = this->metricSubspaceDistance(q1, q2Full, i);
					dist += currSubspaceDist;

					// if an object has to be moved, consider adding an extra penalty to the distance
					if(i > 0 && currSubspaceDist > this->epsilon)
					{
						// add the moving object penalty only if object is on support surface on both configurations or if the robot first has to move to the object which is on a support surface
						bool q1OnSS = this->isOnSupportSurface(q1, i);
						bool q2OnSS = this->isOnSupportSurface(q2Full, i);
						if((q1OnSS && q2OnSS) || (this->forwardSearch && q1OnSS && !q2OnSS) || (!this->forwardSearch && !q1OnSS && q2OnSS))
							dist += this->metMovingObjectPenalty;
					}
				}
				dist = this->transformedDistance(dist);
			}
			else if(this->metType == "ForwardDistanceWithObjectPenalty")
			{
				// Andre's FK metric
				// TODO: copy metricSubspaceDistance(q1, q2Full, 0) in this function, so you don't need to do the double FK twice!
				rl::math::Transform robot_tool_from = this->calcForwardKinematics(q1.segment(0, this->getDof(0)), 0);
				rl::math::Transform robot_tool_to = this->calcForwardKinematics(q2Full.segment(0, this->getDof(0)), 0);
				dist = metricSubspaceDistance(q1, q2Full, 0);
				for(::std::size_t i=1; i<this->getNumMovableComponents(); ++i)
				{
					::std::size_t offset = this->indexFromSubspace(i);
					rl::math::Vector object_from = q1.segment(offset, this->getDof(i));
					rl::math::Vector object_to = q2Full.segment(offset, this->getDof(i));
					rl::math::Real distance_object = (object_to - object_from).norm();
					if(distance_object > this->epsilon)
					{
						dist += this->metMovingObjectPenalty + distance_object;
						rl::math::Real manip_eps = 0.08f;
						rl::math::Real distance_from = (robot_tool_from.translation() - object_from).norm();
						if(distance_from > manip_eps)
						{
							dist += this->metMoveToObjectPenalty + this->metMoveToObjectFactor*distance_from;
						}
						rl::math::Real distance_to = (robot_tool_to.translation() - object_to).norm();
						if(distance_to > manip_eps)
						{
							dist += this->metMoveToObjectPenalty + this->metMoveToObjectFactor*distance_to;
						}
					}
				}
			}
			else if(this->metType == "MiniESPMetricWithPenalties")
			{
				//::std::cout << "okay, here we go ..." << ::std::endl;
				//printQLine(q1, true, true, true, true);
				//printQLine(q2Full, true, true, true, true);
				//while(std::cin.get()!='\n');

				// TODO: consider forward/backward control?!

				// TODO: here we always add the robot traversal from start to goal although it might look stupid
				dist = metricSubspaceDistance(q1, q2Full, 0);

				//::std::cout << "dist is: " << dist << ::std::endl;
				//while(std::cin.get()!='\n');

				::std::vector<bool> vecObjectInGoal(this->getNumMovableComponents(), false);
				::std::vector<double> vecObjectMoveDistance(this->getNumMovableComponents(), 0.0);
				for(::std::size_t i=1; i<this->getNumMovableComponents(); ++i)
				{
					::std::size_t offset = this->indexFromSubspace(i);
					::std::size_t dof = this->getDof(i);
					vecObjectMoveDistance.at(i) = (q1.segment(offset, dof) - q2Full.segment(offset, dof)).norm();
					if(vecObjectMoveDistance.at(i) < this->epsilon)
						vecObjectInGoal.at(i) = true;
				}

				rl::math::Transform robotToolStart = this->calcForwardKinematics(q1.segment(0, this->getDof(0)), 0);
				rl::math::Transform robotToolGoal = this->calcForwardKinematics(q2Full.segment(0, this->getDof(0)), 0);
				rl::math::Vector currLocation = robotToolStart.translation();

				::std::size_t manipSubspace;
				::rl::math::Real bestDistRobotObject = ::std::numeric_limits< double >::max();
				::rl::math::Real currDistRobotObject = bestDistRobotObject;
				bool manipAtLeastOnce = false;
				bool allObjectsInGoal = false;
				while(!allObjectsInGoal)
				{
					bestDistRobotObject = ::std::numeric_limits< double >::max();
					currDistRobotObject = bestDistRobotObject;
					allObjectsInGoal = true;

					// search for the nearest object to be moved
					for(::std::size_t i=1; i<this->getNumMovableComponents(); ++i)
					{
						if(vecObjectInGoal.at(i))
							continue;

						allObjectsInGoal = false;
						::std::size_t offset = this->indexFromSubspace(i);
						::std::size_t dof = this->getDof(i);
						currDistRobotObject = (currLocation - q1.segment(offset, dof)).norm();
						if(currDistRobotObject < bestDistRobotObject)
						{
							bestDistRobotObject = currDistRobotObject;
							manipSubspace = i;
						}
					}

					// found one -> traverse it!
					if(!allObjectsInGoal)
					{
						// TODO: reasonable value ?! YEAH, for meka!! but what about the other robots ?!
						// FIXME: this belons to the xml, also for the forwardMetric, there is the same value !!
						rl::math::Real manipDist = 0.08f;

						::std::size_t offset = this->indexFromSubspace(manipSubspace);
						::std::size_t dof = this->getDof(manipSubspace);

						//::std::cout << "traverse for moving object " << manipSubspace << ::std::endl;
						//while(std::cin.get()!='\n');

						dist += this->metMovingObjectPenalty + vecObjectMoveDistance.at(manipSubspace);

						//::std::cout << "dist is now: " << dist << ::std::endl;
						//while(std::cin.get()!='\n');

						if(currDistRobotObject > manipDist)
						{
							dist += this->metMoveToObjectPenalty + this->metMoveToObjectFactor*currDistRobotObject;

							//::std::cout << "dist is now (after move to object): " << dist << ::std::endl;
							//while(std::cin.get()!='\n');
						}

						currLocation = q2Full.segment(offset, dof);
						vecObjectInGoal.at(manipSubspace) = true;
						manipAtLeastOnce = true;
					}
				}

				if(manipAtLeastOnce)
					dist += (currLocation - robotToolGoal.translation()).norm();

				//::std::cout << "final robot move to goal, dist final: " << dist << ::std::endl;
				//while(std::cin.get()!='\n');
			}
			else
				std::cerr << "DamaModel::transformedDistance: no metric defined." << std::endl;
		}

		return dist;
	}

	::rl::math::Real DamaModel::maxDistanceToRectangle(const ::rl::math::Vector& q, const ::rl::math::Vector& min, const ::rl::math::Vector& max) const
	{
		std::cout << "NOT IMPLEMENTED YET: call to maxDistanceToRectangle(vector): ";
		this->printQLine(q, false, false);
		std::cout << " with min: ";
		this->printQLine(min, false, false);
		std::cout << " with max: ";
		this->printQLine(max, false, true);
		while(std::cin.get()!='\n');

		::rl::math::Real d = 0;

		for (::std::size_t i = 0; i < this->getDof(); ++i)
		{
			if(!this->currIsChosen.at(i))
				continue;

			::rl::math::Real delta = ::std::max(::std::fabs(q(i) - min(i)), ::std::fabs(q(i) - max(i)));

			/*if (this->joints[i]->wraparound)
			{
				::rl::math::Real range = ::std::fabs(this->joints[i]->max - this->joints[i]->min);
				d += this->transformedDistance(::std::max(delta, ::std::fabs(range - delta)));
			}
			else
			{
				d += this->transformedDistance(delta);
			}*/

			d += this->transformedDistance(delta);
		}

		return d;
	}

	::rl::math::Real DamaModel::minDistanceToRectangle(const ::rl::math::Vector& q, const ::rl::math::Vector& min, const ::rl::math::Vector& max) const
	{
		::rl::math::Real d = 0;

		for (::std::size_t i = 0; i < this->getDof(); ++i)
		{
			if(this->currIsChosen.at(i) && (q(i) < min(i) || q(i) > max(i)))
				d += this->transformedDistance(::std::min(::std::fabs(q(i) - min(i)), ::std::fabs(q(i) - max(i))));
		}

		/*std::cout << "call to minDistanceToRectangle(vector): ";
		this->printQLine(q, false, false);
		std::cout << " with min: ";
		this->printQLine(min, false, false);
		std::cout << " with max: ";
		this->printQLine(max, false, false);
		std::cout << " => final dist: " << d << std::endl;
		while(std::cin.get()!='\n');*/

		return d;
	}

	// TODO: now this function will only be called if creating new bucket -> what if q is 0, i.e. not currently chosen ... ? currently just return 0 ...
	::rl::math::Real DamaModel::minDistanceToRectangle(const ::rl::math::Real& q, const ::rl::math::Real& min, const ::rl::math::Real& max, const ::std::size_t& cuttingDimension) const
	{
		//std::cout << "call to minDistanceToRectangle: q = " << q << ", min = " << min << ", max = " << max << ", cuttingDimension = " << cuttingDimension << std::endl;
		//while(std::cin.get()!='\n');

		::rl::math::Real d = 0;

		if (this->currIsChosen.at(cuttingDimension) && (q < min || q > max))
		{
			::rl::math::Real delta = ::std::min(::std::fabs(q - min), ::std::fabs(q - max));

			/*if (this->joints[cuttingDimension]->wraparound)
			{
				::rl::math::Real range = ::std::fabs(this->joints[cuttingDimension]->max - this->joints[cuttingDimension]->min);
				::rl::math::Real size = ::std::fabs(max - min);
				d += ::std::min(delta, ::std::fabs(range - size - delta));
			}
			else
			{
				d += delta;
			}*/

			d += delta;
		}

		return d;
	}

	::rl::math::Real DamaModel::newDistance(const ::rl::math::Real& dist, const ::rl::math::Real& oldOff, const ::rl::math::Real& newOff, const int& cuttingDimension) const
	{
		//std::cout << "call to newDistance" << std::endl;
		//while(std::cin.get()!='\n');

		return dist - this->transformedDistance(oldOff) + this->transformedDistance(newOff);
	}

	::std::size_t DamaModel::indexFromSubspace(::std::size_t subspace) const
	{
		::std::size_t currIndex = 0;

		for(::std::size_t i=0; i<subspace; ++i)
		{
			currIndex += this->getDof(i);
		}

		return currIndex;
	}

	::rl::math::Real DamaModel::metricSubspaceDistance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2, ::std::size_t subspace) const
	{
		if(subspace == 0)
		{
			rl::math::Real distResult = (q1.segment(0, this->getDof(0)) - q2.segment(0, this->getDof(0))).norm();

			if(this->allRevoluteJoints)
			{
				distResult *= this->metRadToMeter;
				rl::math::Transform T1 = this->calcForwardKinematics(q1.segment(0, this->getDof(0)), 0);
				rl::math::Transform T2 = this->calcForwardKinematics(q2.segment(0, this->getDof(0)), 0);
				rl::math::Real distCartesian = (T1.translation() - T2.translation()).norm();
				distResult += distCartesian;
				::rl::math::Quaternion quat1(T1.rotation());
				::rl::math::Quaternion quat2(T2.rotation());
				::rl::math::Real angDist = ::std::fabs(quat1.dot(quat2));
				if(angDist >= 1.0)	// ATTENTION: somehow the Eigen dot product can become a bit larger than 1.0 !!!
					angDist = 0.0;
				else
					angDist = 1.0 - angDist;
				distResult += this->metQuatToMeter * angDist;
				//weight * ::std::pow(2 * ::std::acos(::Eigen::internal::abs(quat1.dot(quat2))), 2);
				// = nearly =
				//weight * ::std::pow(quat1.angularDistance(quat2), 2);
			}

			return (this->metOverallRobotWeight * distResult);
		}
		else
		{
			::std::size_t offset = this->indexFromSubspace(subspace);
			::std::size_t dof = this->getDof(subspace);

			assert(subspace >= this->numRobots && subspace < this->getNumMovableComponents() && this->getDof(subspace) == 3);

			return (q1.segment(offset, dof) - q2.segment(offset, dof)).norm();
		}
	}

	bool DamaModel::isMovingSubspace(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2, ::std::size_t subspace) const
	{
		::std::size_t offset = this->indexFromSubspace(subspace);
		::std::size_t dof = this->getDof(subspace);

		for(::std::size_t i=offset; i<offset+dof; ++i)
			if(::std::fabs(q1(i)-q2(i)) > this->epsilon)
				return true;

		return false;
	}

	bool DamaModel::isMovingObject(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const
	{
		for(::std::size_t i=this->getDof(0); i<q1.size(); ++i)
			if(::std::fabs(q1(i)-q2(i)) > this->epsilon)
				return true;

		return false;
	}

	bool DamaModel::isMoving(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const
	{
		for(::std::size_t i=0; i<q1.size(); ++i)
			if(::std::fabs(q1(i)-q2(i)) > this->epsilon)
				return true;

		return false;
	}

	bool DamaModel::isGoal(const ::rl::math::Vector& q) const
	{
		for(::std::size_t i=0; i<q.size(); ++i)
			if(this->dRrt->goalDimDefined->at(i) && ::std::fabs(q(i)-(*this->dRrt->goal)(i)) > this->epsilon)
				return false;

		return true;
	}

	::rl::math::Real DamaModel::robotMovementDistance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const
	{
		// TODO: maybe implement something more time-related, like the time needed to drive from q1 to q2

		::std::size_t dof = this->getDof(0);
		rl::math::Real distJoints = (q1.segment(0, dof) - q2.segment(0, dof)).norm();

		return distJoints; // TODO: this->metRadianInMeters * ? but only for all-joints-revolute robots
	}

	::rl::math::Real DamaModel::metricConnectInterpolate(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const
	{
		// TODO: maybe implement something more time-related, like the time needed to drive from q1 to q2

		rl::math::Real dist = 0;

		for(::std::size_t i=0; i<1/*this->getNumMovableComponents()*/; ++i)
		{
			::std::size_t offset = this->indexFromSubspace(i);
			::std::size_t dof = this->getDof(i);

			dist += (q1.segment(offset, dof) - q2.segment(offset, dof)).norm();
			if(i == 0 && this->allRevoluteJoints)
				dist *= this->metRadToMeter;
		}

		return dist;
	}

	::rl::math::Real DamaModel::cartesianRobotDistance(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2) const
	{
		rl::math::Transform T1 = this->calcForwardKinematics(q1.segment(0, this->getDof(0)), 0);
		rl::math::Transform T2 = this->calcForwardKinematics(q2.segment(0, this->getDof(0)), 0);

		return (T1.translation() - T2.translation()).norm();
	}

	::rl::math::Real DamaModel::cartesianRobotDistanceToObject(const ::rl::math::Transform& T, const ::rl::math::Vector& q, ::std::size_t subspace) const
	{
		assert(subspace >= this->numRobots && subspace < this->getNumMovableComponents() && this->getDof(subspace) == 3);

		::std::size_t offset = this->indexFromSubspace(subspace);

		return ::std::sqrt(
			::std::pow(T.translation().x() - q(offset+0), 2) +
			::std::pow(T.translation().y() - q(offset+1), 2) +
			::std::pow(T.translation().z() - q(offset+2), 2)
		);
	}

	const ::rl::math::Transform& DamaModel::calcForwardKinematics(const ::rl::math::Vector& q, const ::std::size_t positionIndex, ::rl::mdl::Dynamic* currMdl) const
	{
		if(currMdl == NULL)
			currMdl = this->mdl;

		assert(q.size() == currMdl->getDof());

		currMdl->setPosition(q);
		currMdl->forwardPosition();

		return currMdl->getOperationalPosition(positionIndex);
	}

	bool
	DamaModel::calcInversePositionTaskPosture(::rl::mdl::Kinematic* kin, const ::rl::math::Transform& x_target, ::rl::math::Vector& q_current, const rl::math::Matrix& task_space_projection, const bool coupleJoint1And2, const ::std::size_t& iterations, rl::math::Real limit_q_target_step_norm, rl::math::Real limit_q_posture_step_norm)
	{
		if(coupleJoint1And2)
		{
			return calcInversePositionTaskPostureMeka(kin, x_target, q_current, task_space_projection, iterations, limit_q_target_step_norm, limit_q_posture_step_norm);
		}
		
		std::size_t dof = kin->getDof();

		assert(q_current.size() == dof);

		//TODO currently add a little noise, work-around for degenerate toDelta at exact 180 deg rotation
		::rl::math::Vector noise = ::rl::math::Vector::Ones(q_current.size()) * ::rl::math::DEG2RAD;

		kin->setPosition(q_current + noise);
		::rl::math::Vector dx(6);
		::rl::math::Vector dq(dof);
		::rl::math::Vector qDelta(dof);
		::rl::math::Transform xi;
		::rl::math::Transform x = x_target;
		::rl::math::Real norm = 1;
		::rl::math::Real norm_target = 1.0e-3f;
		::rl::math::Real norm_qDelta;
		rl::math::Vector q_normalized;
		bool warning_written = false;

		/*::std::ofstream myfile;
		myfile.open("iv-kin-new.data", std::ofstream::app);
		myfile << "Calculate IK to T = (";
		rl::math::Transform::TranslationPart position = x.translation();
		rl::math::Vector3 orientation = x.rotation().eulerAngles(2, 1, 0).reverse();
		myfile << position.x() << " | " << position.y() << " | " << position.z() << " || " << (orientation.x() * rl::math::RAD2DEG) << " | " << (orientation.y() * rl::math::RAD2DEG) << " | " << (orientation.z() * rl::math::RAD2DEG);
		myfile << ") starting at q_deg = (" << q_current.matrix().transpose() << ")" << std::endl;*/

		// Posture Optimization (preprocessing steps)

		// Normalize angles on -1..+1
		rl::math::Vector kin_minimum(dof), kin_maximum(dof);
		kin->getMinimum(kin_minimum);
		kin->getMaximum(kin_maximum);

		rl::math::Vector kin_mean = (kin_minimum + kin_maximum) / 2;
		rl::math::Vector kin_range = kin_maximum - kin_minimum;

		for (::std::size_t i = 0; i < iterations; ++i)
		{
			kin->forwardPosition();

			//myfile << "Debug-inversePositionTaskPosture: q_current: " << q_current.transpose() << ::std::endl;

			xi = kin->getOperationalPosition(0);
			::rl::math::transform::toDelta(xi, x, dx, true); // useApproximation true for RL > 0.6.2

			/*::std::cout << ::std::endl;
			::std::cout << "xi: " << ::std::endl;
			::std::cout << xi.matrix().transpose() << ::std::endl;
			::std::cout << "x: " << ::std::endl;
			::std::cout << x.matrix().transpose() << ::std::endl;
			::std::cout << "dx: " << ::std::endl;
			::std::cout << dx.matrix().transpose() << ::std::endl;
			::std::cout << "norm: " << ::std::endl;
			::std::cout << (task_space_projection * dx).norm() << ::std::endl;
			::std::cout << ::std::endl;*/

			kin->calculateJacobian();
			::rl::math::Matrix J = kin->getJacobian();

			if(coupleJoint1And2)
				J.col(1) = J.col(2);

			//myfile << "Debug-my: jacobian: " << J.matrix().transpose() << std::endl;

			kin->calculateJacobianInverse(1e-3, true);
			::rl::math::Matrix invJ = kin->getJacobianInverse();

			// Joint space direction
			dq = invJ * task_space_projection * dx;

			/*myfile << "Debug-my: dx: " << dx.matrix().transpose() << std::endl;
			myfile << "Debug-my: this->invJ: " << invJ.matrix().transpose() << std::endl;
			myfile << "Debug-my: task_space_projection: " << task_space_projection.matrix().transpose() << std::endl;
			myfile << "Debug-my: dq: " << dq.matrix().transpose() << std::endl;*/

			if(coupleJoint1And2)
				dq(1) = dq(2) = (dq(1) + dq(2)) / 2;

			// Limit iteration according to reasonable step size
			rl::math::Real limit_q_step_factor = dq.norm() / limit_q_target_step_norm;
			if(limit_q_step_factor > 1)
				dq /= limit_q_step_factor;
			rl::math::Matrix null_space_projection = rl::math::Matrix::Identity(dof,dof) - (invJ * task_space_projection * J);

			// Posture Optimization
			q_normalized = (q_current - kin_mean).cwiseQuotient(kin_range) * 2;
			rl::math::Vector posture_update = -q_normalized.array() * q_normalized.array().abs() * DamaModel::ikPostureGainMaxStep;
			rl::math::Vector dq_posture = null_space_projection * posture_update;

			if(coupleJoint1And2)
				dq_posture(1) = dq_posture(2) = (dq_posture(1) + dq_posture(2)) / 2;

			//dq_posture += 1 * rl::math::DEG2RAD * posture_update;

			// Limit iteration according to reasonable step size
			rl::math::Real limit_q_posture_step_factor = dq_posture.norm() / limit_q_posture_step_norm;
			if(limit_q_posture_step_factor > 1)
				dq_posture /= limit_q_posture_step_factor;

			qDelta = dq + dq_posture;
			norm_qDelta = qDelta.norm();
	#if 0
			myfile << i << ": q: " << q_current.matrix().transpose() * rl::math::RAD2DEG << std::endl;
			myfile << i << ": dx: " << dx.matrix().transpose() << std::endl;
			//myfile << i << ": this->invJ: " << this->invJ << std::endl;
			//myfile << i << ": task_space_projection: " << task_space_projection << std::endl;
			myfile << i << ": dq: " << dq.matrix().transpose() << std::endl;
			myfile << i << ": qDelta: " << qDelta.matrix().transpose() << std::endl;
			myfile << i << ": norm_qDelta: " << norm_qDelta << std::endl;
			//myfile << i << ": q_normalized: " << q_normalized.matrix().transpose() << std::endl;
			myfile << i << ": posture_update: " << posture_update.matrix().transpose() << std::endl;
			myfile << i << ": dq_posture: " << dq_posture.matrix().transpose() << std::endl;
			myfile << i << ": (task_space_projection * dx).norm(): " << (task_space_projection * dx).norm() << std::endl;
			myfile << i << ": posture_update.norm(): " << posture_update.norm() << std::endl;
	#endif
			/*q_current.matrix().transpose() * rl::math::RAD2DEG;
			dx.matrix().transpose();
			//std::cout << "Debug: this->invJ: " << this->invJ << std::endl;
			//std::cout << "Debug: task_space_projection: " << task_space_projection << std::endl;
			dq.matrix().transpose();
			q_normalized.matrix().transpose();
			posture_update.matrix().transpose();
			dq_posture.matrix().transpose();
			(task_space_projection * dx).norm();
			posture_update.norm();*/
			q_current += qDelta;
			/*if(norm_qDelta < 1e-6 && !warning_written)
			{
				//std::cerr << "Notice: IK in local minimum" << std::endl;	// TODO: uncomment ?!
				std::cout << "Debug: q: " << q_current.matrix().transpose() * rl::math::RAD2DEG << std::endl;
				std::cout << "Debug: dx: " << dx.matrix().transpose() << std::endl;
				//std::cout << "Debug: this->invJ: " << this->invJ << std::endl;
				//std::cout << "Debug: task_space_projection: " << task_space_projection << std::endl;
				std::cout << "Debug: dq: " << dq.matrix().transpose() << std::endl;
				std::cout << "Debug: q_normalized: " << q_normalized.matrix().transpose() << std::endl;
				std::cout << "Debug: posture_update: " << posture_update.matrix().transpose() << std::endl;
				std::cout << "Debug: dq_posture: " << dq_posture.matrix().transpose() << std::endl;
				std::cout << "Debug: (task_space_projection * dx).norm(): " << (task_space_projection * dx).norm() << std::endl;
				std::cout << "Debug: posture_update.norm(): " << posture_update.norm() << std::endl;
				warning_written = true;
			}*/
			kin->setPosition(q_current);
			//std::cout << "q_current: " << q_current.matrix().transpose() * rl::math::RAD2DEG << std::endl;
			norm = (task_space_projection * dx).norm();

			if(norm_qDelta < 1e-8)
			{
				// Local minimum, not sure if valid
				break;
			}

			if(DamaModel::ikPrematureQuit)
			{
				//FIXME: check if joint angles are in valid range ??
				// or: if(norm < 3*norm_target && q_normalized.maxCoeff() < 1 && q_normalized.minCoeff() > -1) ??
				if(norm < norm_target && q_normalized.maxCoeff() < 0.9 && q_normalized.minCoeff() > -0.9)
				{
					// Use without further optimizing posture, generates different grasps wrt start value
					return true; //break;
				}
				/*else
				{
					return false;
				}*/
			}
		}

		/*myfile << "calc-end" << std::endl;
		myfile.close();*/

		if(norm < norm_target)
		{
			return true;
		}

		return false;
	}
	
	bool
	DamaModel::calcInversePositionTaskPostureMeka(::rl::mdl::Kinematic* kin, const ::rl::math::Transform& x_target, ::rl::math::Vector& q_current, const rl::math::Matrix& task_space_projection, const ::std::size_t& iterations, rl::math::Real limit_q_target_step_norm, rl::math::Real limit_q_posture_step_norm)
	{
		std::size_t dof = kin->getDof();

		assert(q_current.size() == dof);

		//currently add a little noise, work-around for degenerate toDelta at exact 180 deg rotation
		::rl::math::Vector noise = ::rl::math::Vector::Ones(q_current.size()) * ::rl::math::DEG2RAD;

		kin->setPosition(q_current + noise);
		::rl::math::Vector dx(6);
		::rl::math::Vector dq(dof);
		::rl::math::Vector qDelta(dof);
		::rl::math::Transform xi;
		::rl::math::Transform x = x_target;
		::rl::math::Real norm = 1;
		::rl::math::Real norm_target = 1.0e-3f;
		::rl::math::Real norm_qDelta;
		rl::math::Vector q_normalized;
		bool warning_written = false;

		// Posture Optimization (preprocessing steps)

		// Normalize angles on -1..+1
		rl::math::Vector kin_minimum(dof), kin_maximum(dof);
		kin->getMinimum(kin_minimum);
		kin->getMaximum(kin_maximum);

		rl::math::Vector kin_mean = (kin_minimum + kin_maximum) / 2;
		rl::math::Vector kin_range = kin_maximum - kin_minimum;

		::rl::math::Matrix motor_to_joints = ::rl::math::Matrix::Zero(10, 9);
		motor_to_joints.topLeftCorner(2, 2) = ::rl::math::Matrix::Identity(2, 2);
		motor_to_joints.bottomRightCorner(8, 8) = ::rl::math::Matrix::Identity(8, 8);
		
		::rl::math::Matrix joints_to_motor = ::rl::math::Matrix::Zero(9, 10);
		joints_to_motor(0,0) = 1;
		joints_to_motor(1,1) = joints_to_motor(1,2) = 0.5;
		joints_to_motor.bottomRightCorner(7, 7) = ::rl::math::Matrix::Identity(7, 7);
		
		//std::cout << "Debug: motor_to_joints: " << std::endl << motor_to_joints.matrix() << std::endl;
		//std::cout << "Debug: joints_to_motor: " << std::endl << joints_to_motor.matrix() << std::endl;
		
		for (::std::size_t i = 0; i < iterations; ++i)
		{
			kin->forwardPosition();

			xi = kin->getOperationalPosition(0);
			::rl::math::transform::toDelta(xi, x, dx, false); // useApproximation for RL > 0.6.2
				
			kin->calculateJacobian();
			::rl::math::Matrix J_motor = kin->getJacobian() * motor_to_joints;
				
			::rl::math::Real tolerance = 1e-8;
			::Eigen::JacobiSVD< ::rl::math::Matrix > svd(J_motor, ::Eigen::ComputeFullU | ::Eigen::ComputeFullV);
			::rl::math::Matrix s = ::rl::math::Matrix::Zero(svd.matrixV().cols(), svd.matrixU().rows());
			for(int i = 0; i < svd.singularValues().size(); ++i)
			{
				::rl::math::Real sing = svd.singularValues()(i);
				s(i,i) = (sing > tolerance) ? (1 / sing) : 0;
			}
			::rl::math::Matrix invJ_motor = svd.matrixV() * s * svd.matrixU().transpose();
			
			// Joint space direction
			::rl::math::Vector dq_motor = invJ_motor * task_space_projection * dx;

			dq = motor_to_joints * dq_motor;
			
			// Limit iteration according to reasonable step size
			rl::math::Real limit_q_step_factor = dq.norm() / limit_q_target_step_norm;
			if(limit_q_step_factor > 1)
				dq /= limit_q_step_factor;
			
			
			rl::math::Matrix null_space_projection = rl::math::Matrix::Identity(9, 9) - (invJ_motor * task_space_projection * J_motor);

			// Posture Optimization
			q_normalized = (q_current - kin_mean).cwiseQuotient(kin_range) * 2;
			rl::math::Vector posture_update = -q_normalized.array() * q_normalized.array().abs() * DamaModel::ikPostureGainMaxStep;
			rl::math::Vector dq_posture = motor_to_joints * null_space_projection * joints_to_motor * posture_update;

			// Limit iteration according to reasonable step size
			rl::math::Real limit_q_posture_step_factor = dq_posture.norm() / limit_q_posture_step_norm;
			if(limit_q_posture_step_factor > 1)
				dq_posture /= limit_q_posture_step_factor;

			qDelta = dq + dq_posture;
			norm_qDelta = qDelta.norm();
			q_current += qDelta;

			kin->setPosition(q_current);
			norm = (task_space_projection * dx).norm();

			if(norm_qDelta < 1e-8)
			{
				// Local minimum, not sure if valid
				break;
			}

			if (DamaModel::ikPrematureQuit)
			{
				//FIXME: check if joint angles are in valid range ??
				// or: if(norm < 3*norm_target && q_normalized.maxCoeff() < 1 && q_normalized.minCoeff() > -1) ??
				if(norm < norm_target && q_normalized.maxCoeff() < 0.9 && q_normalized.minCoeff() > -0.9)
				{
					// Use without further optimizing posture, generates different grasps wrt start value
					return true; //break;
				}
			}
		}

		if(norm < norm_target)
		{
			return true;
		}

		return false;
	}

	/** Note: depends on this->currIsChosen !!! */
	void DamaModel::completePartialSample(const ::rl::math::Vector& samplePart, const ::rl::math::Vector& nearest, ::rl::math::Vector& sampleFull) const
	{
		// complete the values of sample using the values of 'nearest' if current currIsChosen is false
		for(::std::size_t i=0; i<this->getDof(); ++i)
		{
			if(!this->currIsChosen.at(i))
			{
				assert(samplePart(i) == 0.0);
				sampleFull(i) = nearest(i);
			}
			else
			{
				sampleFull(i) = samplePart(i);
			}
		}
	}

	/** Note: depends on this->currIsChosen !!! */
	// TODO: "must be deterministic in the choice of useful primitives for multi-modal hierarchical planning" (-> PHD thesis barry)
	ESP::Res DamaModel::emptySpacePlanner(::std::vector< ::rl::math::Vector >& vecRes, const ::rl::math::Vector& q1, const ::rl::math::Vector& q2, ::std::vector< ::std::string >& vecEdgeAction, const bool usePreciseVersion) const
	{
		//std::cout << "emptySpacePlanner started ..." << std::endl;

		::std::vector< DamaPrim* > vecDamaPrimRand(this->vecDamaPrim);
		// TODO: just test if it yields another result if turned off, and if it is still deterministic inside
		::std::random_shuffle(vecDamaPrimRand.begin(), vecDamaPrimRand.end());

		bool foundUsefulPrim = false;
		::std::vector<bool> vecDamaPrimFailed(this->vecDamaPrim.size(), false);

		const ::rl::math::Vector* qEnd;

		if(this->forwardSearch)
		{
			vecRes.push_back(q1);
			qEnd = &q2;
		}
		else
		{
			vecRes.push_back(q2);
			qEnd = &q1;
		}

		do
		{
			foundUsefulPrim = false;

			for(::std::size_t i=0; i<vecDamaPrimRand.size(); ++i)
			{
				if(!vecDamaPrimFailed.at(i) && vecDamaPrimRand.at(i)->isUseful(vecRes.back(), *qEnd))
				{
					if(this->debugMode)
						std::cout << "* Found useful primitive: " << vecDamaPrimRand.at(i)->getName() << std::endl;
					foundUsefulPrim = true;

					// TODO: somewhat hardcoded here, this config-information can also be stored in the node/vertex itself?
					// TODO: STORE THE INFO (= whether the robot currently is in a grasp, push position of object x etc.) IN THE VERTICES !
					/*::std::size_t qStatus = 0;
					const ::rl::math::Transform T = this->calcForwardKinematics(vecRes.back().segment(0, this->getDof(0)), 0);
					if(usePreciseVersion)
					{
						for(::std::size_t u=this->numRobots; u<this->getNumMovableComponents(); ++u)
							...
								qStatus = u;
					}*/

					if(!vecDamaPrimRand.at(i)->propagate(vecRes, *qEnd, vecEdgeAction))
					{
						vecDamaPrimFailed.at(i) = true;
					}
					else
					{
						std::fill(vecDamaPrimFailed.begin(), vecDamaPrimFailed.end(), false);
					}

					/*for(::std::size_t u=0; u<qProp.size(); ++u)
					{
						std::cout << "u=" << u << ": ";
						this->printQLine(qProp.at(u));
					}*/
				}
			}

			/*std::cout << "~ List of Configuration Points: " << std::endl;
			for(::std::size_t u=0; u<vecRes.size(); ++u)
			{
				std::cout << ">> ";
				this->printQLine(vecRes.at(u), false, false);
				if(u < vecRes.size()-1)
					std::cout << " [-> " << vecEdgeAction.at(u) << "]";
				std::cout << std::endl;
			}*/

		} while(foundUsefulPrim);

		//std::cout << "emptySpacePlanner done." << std::endl;

		// if we searched backwards, reverse the list -> nearestNeighbor is always first, sample is last element
		if(!this->forwardSearch)
		{
			std::reverse(vecRes.begin(), vecRes.end());
			std::reverse(vecEdgeAction.begin(), vecEdgeAction.end());
		}

		// TODO: handle this case not here?! make it elsewhere, refactor and test it !!!!!??
		// If the robot's end position was not specified and the last state was due to 'Transit', skip the last state
		// -> so the robot does not drive to the robot position of the nearest neighbor -> not needed!
		/*if(!this->currIsChosen.at(0) && vecEdgeAction.back().compare(DamaPrimTransit::getInstance()->getName()) == 0)
		{
			vecRes.pop_back();
			vecEdgeAction.pop_back();
		}*/

		/*this->printQLine(vecRes.back(), false, true);
		this->printQLine(*qEnd, false, true);
		::std::cout << this->isMoving(vecRes.back(), *qEnd) << ::std::endl;
		::std::cout << this->getNumMovableComponents() << ::std::endl;
		::std::cout << this->metricSubspaceDistance(vecRes.back(), *qEnd, 2) << ::std::endl;
		::std::cout << this->metricSubspaceDistance(vecRes.back(), *qEnd, 3) << ::std::endl;
		::std::cout << this->epsilon << ::std::endl;*/

		if(this->forwardSearch && this->isMoving(vecRes.back(), *qEnd))
			return ESP::UNREACHABLE;
		else if(!this->forwardSearch && this->isMoving(vecRes.front(), *qEnd))
			return ESP::UNREACHABLE;
		else if(vecRes.size() < 2)
			return ESP::ALREADY_THERE;
		else
			return ESP::OK;
	}

	void DamaModel::interpolate(const ::rl::math::Vector& q1, const ::rl::math::Vector& q2, const ::rl::math::Real& alpha, ::rl::math::Vector& q, ::std::string edgeAction) const
	{
		assert(q1.size() == this->getDof());
		assert(q2.size() == this->getDof());
		assert(alpha >= 0.0f);
		assert(alpha <= 1.0f);
		assert(q.size() == this->getDof());

		for(::std::size_t i=0; i<this->getDof(); ++i)
		{
			q(i) = (1.0f - alpha) * q1(i) + alpha * q2(i);
		}

		if(edgeAction != "")
		{
			// for DamaPrimPickup and DamaPrimTransferRigid make the moved object attached to the grasp pose of the robot arm
			if((edgeAction.substr(0, DamaPrimPickup::getInstance()->getName().size()) == DamaPrimPickup::getInstance()->getName()) || (edgeAction.substr(0, DamaPrimTransferRigid::getInstance()->getName().size()) == DamaPrimTransferRigid::getInstance()->getName()))
			{
				const ::rl::math::Transform T = this->calcForwardKinematics(q.segment(0, this->getDof(0)), 0, this->mdlGrasp);

				::std::size_t primNameObjectNr = boost::lexical_cast< ::std::size_t >(edgeAction.substr(edgeAction.find_last_of(" ")+1));
				::std::size_t startIndex = this->indexFromSubspace(primNameObjectNr);

				//std::cout << "hi, this is " << edgeAction << ", I change object " << primNameObjectNr << " from " << q(startIndex) << ", " << q(startIndex+1) << ", " << q(startIndex+2) << " to " << T.translation().x() << ", " << T.translation().y() << ", " << (T.translation().z() - DamaPrimPickup::HEIGHT_OFFSET_OBJECT) << std::endl;

				q(startIndex) = T.translation().x();
				q(startIndex+1) = T.translation().y();
				q(startIndex+2) = T.translation().z() - DamaPrimPickup::HEIGHT_OFFSET_OBJECT;
			}
		}

		// TODO: under construction by Mr. Gaschler
		//::rl::math::Transform toolQ1 = this->calcForwardKinematics(q1.segment(0, this->getDof(0)), 0, this->mdlGrasp);
		//::rl::math::Transform toolQ = this->calcForwardKinematics(q.segment(0, this->getDof(0)), 0, this->mdlGrasp);
		//::rl::math::Transform toolMotion = toolQ * toolQ1.inverse();
	}

	void DamaModel::setPosition(const ::rl::math::Vector& q)
	{
		for(::std::size_t i=0; i<this->getNumMovableComponents(); ++i)
			this->setPosition(q, i);
	}

	void DamaModel::setPosition(const ::rl::math::Vector& q, ::std::size_t subspace)
	{
		if(subspace == 0)
		{
			if(NULL != kin)
			{
				assert(this->getSubspace(q, 0).size() == this->kin->getDof());
				this->kin->setPosition(this->getSubspace(q, 0));
			}
			else
			{
				assert(this->getSubspace(q, 0).size() == this->mdl->getDof());

				if(!this->isViewerModel)
				{
					/*::std::ofstream myfile("iv-kin.data", std::ofstream::app);
					myfile.precision(18);
					myfile << "Debug-DamaModel: setPosition start" << ::std::endl;*/
					this->mdl->setPosition(this->getSubspace(q, 0));
					//myfile << "Debug-DamaModel: setPosition end" << ::std::endl;
				}
				else
				{
					this->mdl->setPosition(this->getSubspace(q, 0));
				}
			}
		}
		else
		{
			// draw the manipulable object
			::rl::math::Transform currT;
			this->scene->getModel(subspace)->getBody(0)->getFrame(currT);
			::std::size_t startIndex = this->indexFromSubspace(subspace);
			::rl::math::Translation t(q(startIndex), q(startIndex+1), q(startIndex+2));
			currT.translation() = t.vector();
			this->scene->getModel(subspace)->getBody(0)->setFrame(currT);
		}
	}

	void DamaModel::updateFrames(const bool& doUpdateModel)
	{
		if(NULL != kin)
		{
			assert(this->model->getNumBodies() == this->kin->getBodies());

			this->kin->updateFrames();

			if (doUpdateModel)
			{
				for (::std::size_t i = 0; i < this->model->getNumBodies(); ++i)
				{
					this->model->getBody(i)->setFrame(this->kin->getFrame(i));
				}
			}
		}
		else
		{
			assert(this->model->getNumBodies() == this->mdl->getBodies());

			this->mdl->forwardPosition();

			if (doUpdateModel)
			{
				for (::std::size_t i = 0; i < this->model->getNumBodies(); ++i)
				{
					this->model->getBody(i)->setFrame(this->mdl->getFrame(i));
				}
			}
		}
	}

	bool DamaModel::isColliding(::std::string primName)
	{
		++this->totalQueries;

		//this->checkCollisionRobotObjects = false;

		/*if ROBOT == 3 && SCENARIO > 1 //Andre's collision routine
		::rl::sg::SimpleScene* collision_scene = dynamic_cast< ::rl::sg::SimpleScene* >(this->scene);
		::rl::sg::Model *robot_model, *package1_model, *package2_model, *package3_model, *workspace_model;
		robot_model = this->scene->getModel(0);
		package1_model = this->scene->getModel(1);
		package2_model = this->scene->getModel(2);
		if(numObjects >= 3)
		{
			package3_model = this->scene->getModel(3);
			workspace_model = this->scene->getModel(4);
		}
		else
		{
			workspace_model = this->scene->getModel(3);
		}
		//robot must not collide with itself, with an object or environment
		for (int k = 0; k < robot_model->getNumBodies(); ++k)
		{
			for (int l = k + 1; l < robot_model->getNumBodies(); ++l)
			{
				if(this->mdl->areColliding(k, l))
				{
					if(collision_scene->areColliding(robot_model->getBody(k), robot_model->getBody(l))) return true;
				}
			}
		}
		//robot must not collide with an object or environment
		if(collision_scene->areColliding(robot_model, workspace_model)) return true;
		if(primName != "Transfer-Rigid Object 1" && primName != "Pickup Object 1")
		{
			if(collision_scene->areColliding(robot_model, package1_model)) return true;
		}
		if(primName != "Transfer-Rigid Object 2" && primName != "Pickup Object 2")
		{
			if(collision_scene->areColliding(robot_model, package2_model)) return true;
		}
		if(numObjects >= 3)
		{
			if(primName != "Transfer-Rigid Object 3" && primName != "Pickup Object 3")
			{
				if(collision_scene->areColliding(robot_model, package3_model)) return true;
			}
			if(collision_scene->areColliding(package1_model, package2_model)) return true;
			if(collision_scene->areColliding(package2_model, package3_model)) return true;
			if(collision_scene->areColliding(package1_model, package3_model)) return true;
		}
		else
		{
			//objects must not collide with other objects or environment
			if(collision_scene->areColliding(package1_model, package2_model)) return true;
		}
		return false;*/


		// set the right end-effector body depending on the current active primitive
		::std::size_t indexNumber = 0;
		::std::string primNameRaw = DamaPrimTransit::getInstance()->getName();
		::std::size_t primNameObjectNr = 0;
		if(primName.compare(DamaPrimTransit::getInstance()->getName()) != 0)
		{
			indexNumber = primName.find_last_of(" ");
			primNameRaw = primName.substr(0,indexNumber);
			primNameObjectNr = boost::lexical_cast< ::std::size_t >(primName.substr(indexNumber+1));
		}
		//::std::cout << "process primitive " << primNameRaw << ::std::endl;
		::rl::sg::Body* myBody = NULL;
		for(int i=0; i<this->vecDamaPrim.size(); i++)
		{
			//::std::cout << dModel2->vecDamaPrim.at(i)->getName() << " == " << primNameRaw << " ??"  << ::std::endl;
			if(this->vecDamaPrim.at(i)->getName() == primNameRaw)
			{
				myBody = this->vecDamaPrim.at(i)->endEffectorBody;
				break;
			}
		}
		if(myBody != this->currEndEffectorBody)
		{
			//::std::cout << "isColliding: change to end-effector body " << myBody->getName() << ::std::endl;
			this->scene->getModel(0)->remove(this->scene->getModel(0)->getBody(this->scene->getModel(0)->getNumBodies()-1));
			this->scene->getModel(0)->add(myBody);
			this->currEndEffectorBody = myBody;
		}
		//::std::cout << "#bodies: " << dModel2->getBodies() << ::std::endl;
		//::std::cout << "#shapes of last body '" << dModel2->getBody(dModel2->getBodies())->getName() << "': " << dModel2->getBody(dModel2->getBodies())->getNumShapes() << ::std::endl;
		//::std::cout << "#shapes of second last body '" << dModel2->getBody(dModel2->getBodies()-1)->getName() << "': " << dModel2->getBody(dModel2->getBodies()-1)->getNumShapes() << ::std::endl;
		//::rl::sg::so::Scene* sceneVis = static_cast< ::rl::sg::so::Scene* >(dModel2->scene);
		//MainWindow::instance()->viewer->viewer->setSceneGraph(sceneVis->root);




		// go through bodies of my model (box-2d-050505): link0, link1, body1
		for (::std::size_t i = 0; i < this->model->getNumBodies(); ++i)
		{
			if (this->isColliding(i))
			{
				// go through: box-2d-050505, package1, package2, package3, boxes
				for (::rl::sg::Scene::Iterator j = this->scene->begin(); j != this->scene->end(); ++j)
				{
					// only check collision of the own model with the model 'boxes'
					::rl::sg::Model* currModel = *j;
					if (this->model != *j)
					{
						if(this->checkCollisionRobotObjects || (!this->checkCollisionRobotObjects && (currModel->getName() != "package1" && currModel->getName() != "package2" && currModel->getName() != "package3")))
						{
							// go through bodies of the current model: ...
							for (::rl::sg::Model::Iterator k = (*j)->begin(); k != (*j)->end(); ++k)
							{
								if (dynamic_cast< ::rl::sg::SimpleScene* >(this->scene)->areColliding(this->model->getBody(i), *k))
								{
									this->body = i;
									return true;
								}
							}
						}
					}
				}
			}

			// check for self-collision of the bodies of the own model

			for (::std::size_t j = 0; j < i; ++j)
			{
				if (this->areColliding(i, j))
				{
					if (dynamic_cast< ::rl::sg::SimpleScene* >(this->scene)->areColliding(this->model->getBody(i), this->model->getBody(j)))
					{
						this->body = i;
						return true;
					}
				}
			}
		}

		// if not using the primitive "Transit", then also check for object-collisions
		if(primName.compare(DamaPrimTransit::getInstance()->getName()) != 0)
		{
			// moved object (primNameObjectNr) should not collide with the other ones and also not with the scene
			for(::std::size_t j=this->numRobots; j<this->getNumMovableComponents(); ++j)
			{
				if(primNameObjectNr != j && dynamic_cast< ::rl::sg::SimpleScene* >(this->scene)->areColliding(this->scene->getModel(primNameObjectNr), this->scene->getModel(j)))
					return true;
			}

			if(dynamic_cast< ::rl::sg::SimpleScene* >(this->scene)->areColliding(this->scene->getModel(primNameObjectNr), this->scene->getModel(this->getNumMovableComponents())))
				return true;
		}

		this->body = this->getBodies();
		++this->freeQueries;
		return false;
	}

	bool DamaModel::isCollidingWithScene(::std::size_t subspace)
	{
		++this->totalQueries;
		if(dynamic_cast< ::rl::sg::SimpleScene* >(this->scene)->areColliding(this->scene->getModel(subspace), this->scene->getModel(this->getNumMovableComponents())))
			return true;
		++this->freeQueries;
		return false;
	}

	bool DamaModel::isOnSupportSurface(const ::rl::math::Vector& q, const ::std::size_t subspace, ::std::size_t* supportSurface) const
	{
		::std::size_t offset = this->indexFromSubspace(subspace);

		for(::std::size_t s=0; s<this->vecSupportSurface.size(); ++s)
		{
			// TODO: epsilon should be higher than this->epsilon, otherwise I had the case of transferring without picking up before, due to "false" intermediate vertices
			if((::std::fabs(q(offset+2) - this->vecSupportSurface.at(s)->getHeight()) < 1.0e-4f) &&
				(q(offset) >= this->vecSupportSurface.at(s)->getMin(0)-this->epsilon && q(offset) <= this->vecSupportSurface.at(s)->getMax(0)+this->epsilon) &&
				(q(offset+1) >= this->vecSupportSurface.at(s)->getMin(1)-this->epsilon && q(offset+1) <= this->vecSupportSurface.at(s)->getMax(1)+this->epsilon))
			{
				if(supportSurface != NULL)
					*supportSurface = s;
				return true;
			}
		}

		return false;
	}

	void DamaModel::updateRobotPosition(::rl::math::Vector& q, ::rl::mdl::Dynamic* myMdl)
	{
		rl::math::Vector robotPos(this->getDof(0));// = q.segment(0, );
		if(myMdl == NULL)
			this->mdl->getPosition(robotPos);
		else
			myMdl->getPosition(robotPos);
		q.segment(0, this->getDof(0)) = robotPos;
	}
}
