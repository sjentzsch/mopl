/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#include <boost/lexical_cast.hpp>

#include "DamaPrimPickup.h"
#include "DamaPrimTransit.h"
#include "DamaModel.h"

namespace dama
{
	// default values
	rl::math::Real DamaPrimPickup::HEIGHT_OFFSET_OBJECT = 0;
	rl::math::Real DamaPrimPickup::HEIGHT_PICKUP_DIST_OBJECT = 0.10;
	rl::math::Real DamaPrimPickup::MIN_PLANE_DIST_FOR_PICKUP = 0.20;
	rl::math::Real DamaPrimPickup::PICKUP_Z_AXIS = 45 * rl::math::DEG2RAD;
	rl::math::Real DamaPrimPickup::PICKUP_Y_AXIS = 90 * rl::math::DEG2RAD;
	rl::math::Real DamaPrimPickup::PICKUP_X_AXIS = 180 * rl::math::DEG2RAD;

	::std::string DamaPrimPickup::getName() const
	{
		return "Pickup Object";
	}

	bool DamaPrimPickup::isUseful(const ::rl::math::Vector& startState, const ::rl::math::Vector& endState)
	{
		bool foundObjectForPickup = false;

		for(::std::size_t i=dModel->numRobots; i<dModel->getNumMovableComponents(); ++i)
		{
			::std::size_t offset = this->dModel->indexFromSubspace(i);
			::rl::math::Real distPlaneTransformed = ::std::pow(startState(offset+0) - endState(offset+0), 2) + ::std::pow(startState(offset+1) - endState(offset+1), 2);
			::rl::math::Real distHeight = ::std::fabs(startState(offset+2) - endState(offset+2));

			if(this->dModel->isOnSupportSurface(startState, i))
			{
				if(distHeight > 1.0e-4f/*dModel->epsilon*/ || dModel->inverseOfTransformedDistance(distPlaneTransformed) > DamaPrimPickup::MIN_PLANE_DIST_FOR_PICKUP)
					foundObjectForPickup = true;
			}
			else
				return false;
		}

		return foundObjectForPickup;
	}

	bool DamaPrimPickup::propagate(::std::vector< ::rl::math::Vector >& vecRes, const ::rl::math::Vector& endState, ::std::vector< ::std::string >& vecEdgeAction)
	{
		dama::Timer timerIK;

		::std::vector< ::rl::math::Vector > vecResSim = vecRes;
		::std::vector< ::std::string > vecEdgeActionSim = vecEdgeAction;
		::rl::math::Vector startState = vecResSim.back();

		// choose the object which to move depending on the robot travel distance to the location of the respective object
		// TODO: solve traveling salesman problem? Or maybe a bit better (but more time consuming): take pushing pose instead of object's location
		::std::size_t subspacePushObject;
		::rl::math::Real bestDistRobotObject = ::std::numeric_limits< double >::max();
		::rl::math::Real currDistRobotObject;
		const ::rl::math::Transform T = dModel->calcForwardKinematics(startState.segment(0, dModel->getDof(0)), 0, this->dModel->mdlGrasp);
		for(::std::size_t i=dModel->numRobots; i<dModel->getNumMovableComponents(); ++i)
		{
			::std::size_t offset = this->dModel->indexFromSubspace(i);
			::rl::math::Real distPlaneTransformed = ::std::pow(startState(offset+0) - endState(offset+0), 2) + ::std::pow(startState(offset+1) - endState(offset+1), 2);
			::rl::math::Real distHeight = ::std::fabs(startState(offset+2) - endState(offset+2));

			if(distHeight > 1.0e-4f/*dModel->epsilon*/ || dModel->inverseOfTransformedDistance(distPlaneTransformed) > DamaPrimPickup::MIN_PLANE_DIST_FOR_PICKUP)
			{
				currDistRobotObject = dModel->cartesianRobotDistanceToObject(T, startState, i);
				if(currDistRobotObject < bestDistRobotObject)
				{
					bestDistRobotObject = currDistRobotObject;
					subspacePushObject = i;
				}
			}
		}
		::std::size_t indexObject = dModel->indexFromSubspace(subspacePushObject);

		if(this->dModel->debugMode)
			::std::cout << "DamaPrimPickup::propagate: pickup object nr. " << (subspacePushObject) << " from (" << startState(indexObject) << ", " << startState(indexObject+1) << ", " << startState(indexObject+2) << ") towards (" << endState(indexObject) << ", " << endState(indexObject+1) << ", " << endState(indexObject+2) << ")" << std::endl;

		// calculate the robot grasp start position
		rl::math::Transform T1_grasp;
		T1_grasp.translation().x() = startState(indexObject);
		T1_grasp.translation().y() = startState(indexObject+1);
		T1_grasp.translation().z() = startState(indexObject+2) + DamaPrimPickup::HEIGHT_OFFSET_OBJECT;
		T1_grasp.linear() = (
			rl::math::AngleAxis(DamaPrimPickup::PICKUP_Z_AXIS, rl::math::Vector3::UnitZ()) *	// WILL BE FREE AXIS
			rl::math::AngleAxis(DamaPrimPickup::PICKUP_Y_AXIS, rl::math::Vector3::UnitY()) *
			rl::math::AngleAxis(DamaPrimPickup::PICKUP_X_AXIS, rl::math::Vector3::UnitX())
		).toRotationMatrix();

		rl::math::Matrix66 constraint_position_orient = rl::math::Matrix66::Zero();
		constraint_position_orient.diagonal() << 1, 1, 1, 1, 1, 0;

		rl::math::Vector q_ik_start1 = startState.segment(0, dModel->getDof(0));
		if(this->dModel->debugMode)
		{
			::std::cout << "Calculate IK1 to T = (";
			this->dModel->printTransform(T1_grasp, false);
			::std::cout << ") starting at q_deg = (";
			this->dModel->printQLine(q_ik_start1 * rl::math::RAD2DEG, false, false, false, false);
			::std::cout << ")" << ::std::endl;
		}
		timerIK.start();
		bool calcInverse1a = DamaModel::calcInversePositionTaskPosture(dModel->mdlGrasp, T1_grasp, q_ik_start1, constraint_position_orient, this->dModel->coupleJoint1And2);
		timerIK.stop();
		this->dModel->timeIK += timerIK.elapsed();
		if(!calcInverse1a)
		{
			if(this->dModel->debugMode)
				::std::cout << "Result: IK1 failed. Exit." << ::std::endl;
			return false;
		}

		::rl::math::Vector interConf1(startState);
		this->dModel->updateRobotPosition(interConf1, this->dModel->mdlGrasp);

		if(this->dModel->debugMode)
		{
			::std::cout << "Result: IK1 succeeded with q_deg = (";
			this->dModel->printQLine(interConf1, false, false, false, true);
			::std::cout << ")" << ::std::endl;
		}

		// first move to the pushing position if not already there
		if(dModel->isMovingSubspace(startState, interConf1, 0))
		{
			// TODO: investigate why algorithm currently needs about 5.7% more time just by using the uncommented code instead of the one in comments?!
			//vecResSim.push_back(interConf1);
			//vecEdgeActionSim.push_back(DamaPrimTransit::getInstance()->getName());
			DamaPrimTransit::getInstance()->propagate(vecResSim, interConf1, vecEdgeActionSim);
		}

		// calculate the robot pickup end position
		rl::math::Transform T2_pickup;
		T2_pickup.translation().x() = startState(indexObject);
		T2_pickup.translation().y() = startState(indexObject+1);
		T2_pickup.translation().z() = startState(indexObject+2) + DamaPrimPickup::HEIGHT_PICKUP_DIST_OBJECT + DamaPrimPickup::HEIGHT_OFFSET_OBJECT;
		T2_pickup.linear() = (
			rl::math::AngleAxis(DamaPrimPickup::PICKUP_Z_AXIS, rl::math::Vector3::UnitZ()) *	// WILL BE FREE AXIS
			rl::math::AngleAxis(DamaPrimPickup::PICKUP_Y_AXIS, rl::math::Vector3::UnitY()) *
			rl::math::AngleAxis(DamaPrimPickup::PICKUP_X_AXIS, rl::math::Vector3::UnitX())
		).toRotationMatrix();

		rl::math::Vector q_ik_start2 = interConf1.segment(0, dModel->getDof(0));

		if(this->dModel->debugMode)
		{
			::std::cout << "Calculate IK2 to T = (";
			this->dModel->printTransform(T2_pickup, false);
			::std::cout << ") starting at q_deg = (";
			this->dModel->printQLine(q_ik_start2 * rl::math::RAD2DEG, false, false, false, false);
			::std::cout << ")" << ::std::endl;
		}

		timerIK.start();
		bool calcInverse2 = DamaModel::calcInversePositionTaskPosture(dModel->mdlGrasp, T2_pickup, q_ik_start2, constraint_position_orient, this->dModel->coupleJoint1And2);
		timerIK.stop();
		this->dModel->timeIK += timerIK.elapsed();

		if(!calcInverse2)
		{
			if(this->dModel->debugMode)
				::std::cout << "Result: IK2 failed. Exit." << ::std::endl;
			return false;
		}

		// now pickup the object
		::rl::math::Vector interConf2(startState);
		this->dModel->updateRobotPosition(interConf2, this->dModel->mdlGrasp);

		if(this->dModel->debugMode)
		{
			::std::cout << "Result: IK2 succeeded with q_deg = (";
			this->dModel->printQLine(interConf2, false, false, false, true);
			::std::cout << ")" << ::std::endl;
		}

		interConf2(indexObject+2) += DamaPrimPickup::HEIGHT_PICKUP_DIST_OBJECT;
		vecResSim.push_back(interConf2);
		vecEdgeActionSim.push_back(this->getName() + " " + boost::lexical_cast<std::string>(subspacePushObject));

		vecRes = vecResSim;
		vecEdgeAction = vecEdgeActionSim;

		/*::rl::math::Vector* test = NULL;
		test->normalize();*/

		return true;
	}

	::rl::math::Transform DamaPrimPickup::getToolObjectTransform()
	{
		rl::math::Transform T;

		// TODO: do it!
		/*T.translation().x() = 0;
		T.translation().y() = 0;
		T.translation().z() = DamaPrimPickup::HEIGHT_OFFSET_OBJECT;
		T.linear() = (
			rl::math::AngleAxis(DamaPrimPickup::PICKUP_Z_AXIS, rl::math::Vector3::UnitZ()) *	// WILL BE FREE AXIS
			rl::math::AngleAxis(DamaPrimPickup::PICKUP_Y_AXIS, rl::math::Vector3::UnitY()) *
			rl::math::AngleAxis(DamaPrimPickup::PICKUP_X_AXIS, rl::math::Vector3::UnitX())
		).toRotationMatrix();*/

		return T;
	}
}
