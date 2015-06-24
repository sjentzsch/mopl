/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#include <boost/lexical_cast.hpp>

#include "DamaPrimPushExterior.h"
#include "DamaPrimTransit.h"
#include "DamaModel.h"

namespace dama
{
	::std::string DamaPrimPushExterior::getName() const
	{
		return "Push Exterior Object";
	}

	bool DamaPrimPushExterior::isUseful(const ::rl::math::Vector& startState, const ::rl::math::Vector& endState)
	{
		// TODO: Add some constraint, pushing with exterior hand surface only when object moves to the right?!

		bool foundObjectForPush = false;

		for(::std::size_t i=dModel->numRobots; i<dModel->getNumMovableComponents(); ++i)
		{
			::std::size_t offset = this->dModel->indexFromSubspace(i);
			::rl::math::Real distPlane = ::std::fabs(startState(offset+0) - endState(offset+0)) + ::std::fabs(startState(offset+1) - endState(offset+1));
			::rl::math::Real distHeight = ::std::fabs(startState(offset+2) - endState(offset+2));

			if(this->dModel->isOnSupportSurface(startState, i))
			{
				if(distPlane > dModel->epsilon && distHeight < dModel->epsilon)
					foundObjectForPush = true;
			}
			else
				return false;
		}

		return foundObjectForPush;
	}

	bool DamaPrimPushExterior::propagate(::std::vector< ::rl::math::Vector >& vecRes, const ::rl::math::Vector& endState, ::std::vector< ::std::string >& vecEdgeAction)
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
		const ::rl::math::Transform T = dModel->calcForwardKinematics(startState.segment(0, dModel->getDof(0)), 0);
		for(::std::size_t i=dModel->numRobots; i<dModel->getNumMovableComponents(); ++i)
		{
			::std::size_t offset = this->dModel->indexFromSubspace(i);
			::rl::math::Real distPlane = ::std::fabs(startState(offset+0) - endState(offset+0)) + ::std::fabs(startState(offset+1) - endState(offset+1));
			::rl::math::Real distHeight = ::std::fabs(startState(offset+2) - endState(offset+2));

			if(distPlane > dModel->epsilon && distHeight < dModel->epsilon)
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
			::std::cout << "DamaPrimPushExterior::propagate: pushing object nr. " << (subspacePushObject) << " from (" << startState(indexObject) << ", " << startState(indexObject+1) << ", " << startState(indexObject+2) << ") to (" << endState(indexObject) << ", " << endState(indexObject+1) << ", " << endState(indexObject+2) << ")" << std::endl;

		// calculate the 2D surface directional vector from object start to object goal position
		::rl::math::Vector dirVecMovingObject(2);
		dirVecMovingObject(0) = endState(indexObject) - startState(indexObject);
		dirVecMovingObject(1) = endState(indexObject+1) - startState(indexObject+1);
		::rl::math::Real lengthMovingObject = dirVecMovingObject.norm();

		// calculate the real endstate, because we have a limited push distance -> endstate might get cut
		::rl::math::Vector realEndState(3);
		for(::std::size_t i=0; i<3; ++i)
			realEndState(i) = endState(indexObject+i);
		if(lengthMovingObject > DamaPrimPush::MAX_PUSH_DIST)
		{
			realEndState(0) = startState(indexObject) + dirVecMovingObject(0)/lengthMovingObject*DamaPrimPush::MAX_PUSH_DIST;
			realEndState(1) = startState(indexObject+1) + dirVecMovingObject(1)/lengthMovingObject*DamaPrimPush::MAX_PUSH_DIST;

			if(this->dModel->debugMode)
				::std::cout << "Adjusting goal (cut from " << lengthMovingObject << " down to " << DamaPrimPush::MAX_PUSH_DIST << " length): (" << realEndState(0) << ", " << realEndState(1) << ", " << realEndState(2) << ")" << ::std::endl;
		}

		// TODO: uncomment for mobile robot!
		/*
		dirVecMovingObject.normalize();
		dirVecMovingObject *= this->dModel->radiusObject + this->dModel->radiusRobot;
		// ...
		interConf1(0) = startState(indexObject) - dirVecMovingObject(0);
		interConf1(1) = startState(indexObject+1) - dirVecMovingObject(1);
		// ...
		interConf2(0) = endState(indexObject) - dirVecMovingObject(0);
		interConf2(1) = endState(indexObject+1) - dirVecMovingObject(1);
		*/

		// TODO: check if the following still holds for a mobile robot

		// calculate the rotational angle for the end-effector in order to push the object from the right angle
		double theta = atan(- dirVecMovingObject(0) / dirVecMovingObject(1));
		double theta_in = theta;
		if(dirVecMovingObject(1) >= 0.0)
			theta_in += M_PI;
		double theta_ex = theta;
		if(dirVecMovingObject(1) < 0.0)
			theta_ex += M_PI;
		// TODO: following two maybe not needed -> test!
		/*if(theta < 0)
			theta += 2*M_PI;
		if(theta >= 2*M_PI)
			theta -= 2*M_PI;*/

		if(this->dModel->debugMode)
			::std::cout << "#### theta_ex: " << theta_ex << " (rad), resp. " << (theta_ex * rl::math::RAD2DEG) << " (deg)" << ::std::endl;

		// calculate the robot pushing start position
		rl::math::Transform T1_ex;
		T1_ex.translation().x() = startState(indexObject) - DamaPrimPush::DIST_TO_OBJECT_XY*sin(theta_in);
		T1_ex.translation().y() = startState(indexObject+1) + DamaPrimPush::DIST_TO_OBJECT_XY*cos(theta_in);
		T1_ex.translation().z() = startState(indexObject+2) + DamaPrimPush::HEIGHT_OFFSET_OBJECT;
		T1_ex.linear() = (
			rl::math::AngleAxis(theta_ex, rl::math::Vector3::UnitZ()) *
			rl::math::AngleAxis(this->TILT_HAND_INWARD_ANGLE, rl::math::Vector3::UnitX()) *
			rl::math::AngleAxis(this->TILT_HAND_DOWNWARD_ANGLE, rl::math::Vector3::UnitY()) *
			rl::math::AngleAxis(180 * rl::math::DEG2RAD, rl::math::Vector3::UnitX())
		).toRotationMatrix();

		rl::math::Matrix66 constraint_position_orient = rl::math::Matrix66::Zero();
		constraint_position_orient.diagonal() << 1, 1, 1, 1, 1, 1;

		rl::math::Vector q_ik_start1 = startState.segment(0, dModel->getDof(0));
		if(this->dModel->debugMode)
		{
			::std::cout << "Calculate IK1 to T = (";
			this->dModel->printTransform(T1_ex, false);
			::std::cout << ") starting at q_deg = (";
			this->dModel->printQLine(q_ik_start1 * rl::math::RAD2DEG, false, false, false, false);
			::std::cout << ")" << ::std::endl;
		}
		timerIK.start();
		bool calcInverse1a = DamaModel::calcInversePositionTaskPosture(dModel->mdl, T1_ex, q_ik_start1, constraint_position_orient, this->dModel->coupleJoint1And2);
		timerIK.stop();
		this->dModel->timeIK += timerIK.elapsed();
		if(!calcInverse1a)
		{
			if(this->dModel->debugMode)
				::std::cout << "Result: IK1 failed. Exit." << ::std::endl;
			return false;
		}

		::rl::math::Vector interConf1(startState);
		this->dModel->updateRobotPosition(interConf1);

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

		// calculate the robot pushing end position via IK
		rl::math::Transform T2_ex;
		T2_ex.translation().x() = realEndState(0) - DamaPrimPush::DIST_TO_OBJECT_XY*sin(theta_in);
		T2_ex.translation().y() = realEndState(1) + DamaPrimPush::DIST_TO_OBJECT_XY*cos(theta_in);
		T2_ex.translation().z() = realEndState(2) + DamaPrimPush::HEIGHT_OFFSET_OBJECT;
		T2_ex.linear() = (
			rl::math::AngleAxis(theta_ex, rl::math::Vector3::UnitZ()) *
			rl::math::AngleAxis(this->TILT_HAND_INWARD_ANGLE, rl::math::Vector3::UnitX()) *
			rl::math::AngleAxis(this->TILT_HAND_DOWNWARD_ANGLE, rl::math::Vector3::UnitY()) *
			rl::math::AngleAxis(180 * rl::math::DEG2RAD, rl::math::Vector3::UnitX())
		).toRotationMatrix();

		rl::math::Vector q_ik_start2 = interConf1.segment(0, dModel->getDof(0));

		if(this->dModel->debugMode)
		{
			::std::cout << "Calculate IK2 to T = (";
			this->dModel->printTransform(T2_ex, false);
			::std::cout << ") starting at q_deg = (";
			this->dModel->printQLine(q_ik_start2 * rl::math::RAD2DEG, false, false, false, false);
			::std::cout << ")" << ::std::endl;
		}

		timerIK.start();
		bool calcInverse2 = DamaModel::calcInversePositionTaskPosture(dModel->mdl, T2_ex, q_ik_start2, constraint_position_orient, this->dModel->coupleJoint1And2);
		timerIK.stop();
		this->dModel->timeIK += timerIK.elapsed();

		if(!calcInverse2)
		{
			if(this->dModel->debugMode)
				::std::cout << "Result: IK2 failed. Exit." << ::std::endl;
			return false;
		}

		// now push the object to its end position
		::rl::math::Vector interConf2(startState);
		this->dModel->updateRobotPosition(interConf2);

		// check for maximum joint space distance
		::rl::math::Real robotJointPushDistance = (interConf2.segment(0, dModel->getDof(0)) - interConf1.segment(0, dModel->getDof(0))).norm();
		if(robotJointPushDistance > DamaPrimPush::MAX_PUSH_DIST_JOINT)
		{
			if(this->dModel->debugMode)
				::std::cout << "Result: (interConf2 - interConf1).norm() > DamaPrimPushExterior::MAX_PUSH_DIST_JOINT, factor " << (DamaPrimPush::MAX_PUSH_DIST_JOINT / robotJointPushDistance) << std::endl;

			return false;
			//TODO: try to fix (would also need to fix cartesian sample) <-- ask Andre!
			//interConf2 = factor * (interConf2 - interConf1) + interConf1;
		}

		if(this->dModel->debugMode)
		{
			::std::cout << "Result: IK2 succeeded with q_deg = (";
			this->dModel->printQLine(interConf2, false, false, false, true);
			::std::cout << ")" << ::std::endl;
		}

		interConf2(indexObject) = realEndState(0);
		interConf2(indexObject+1) = realEndState(1);
		vecResSim.push_back(interConf2);
		vecEdgeActionSim.push_back(this->getName() + " " + boost::lexical_cast<std::string>(subspacePushObject));

		vecRes = vecResSim;
		vecEdgeAction = vecEdgeActionSim;

		/*::rl::math::Vector* test = NULL;
		test->normalize();*/

		return true;
	}

	::rl::math::Transform DamaPrimPushExterior::getToolObjectTransform()
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
