/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#include <boost/lexical_cast.hpp>

#include "DamaPrimPushMobile.h"
#include "DamaPrimTransit.h"
#include "DamaModel.h"

namespace dama
{
	::std::string DamaPrimPushMobile::getName() const
	{
		return "Push Mobile Object";
	}

	bool DamaPrimPushMobile::isUseful(const ::rl::math::Vector& startState, const ::rl::math::Vector& endState)
	{
		// TODO: only inspect the distance on a surface

		// TODO: Add some constraint, pushing with interior hand surface only when object moves to the left?!

		return dModel->isMovingObject(startState, endState);
	}

	bool DamaPrimPushMobile::propagate(::std::vector< ::rl::math::Vector >& vecRes, const ::rl::math::Vector& endState, ::std::vector< ::std::string >& vecEdgeAction)
	{
		dama::Timer timerIK;

		::std::vector< ::rl::math::Vector > vecResSim = vecRes;
		::std::vector< ::std::string > vecEdgeActionSim = vecEdgeAction;
		::rl::math::Vector startState = vecResSim.back();

		// TODO: only inspect the distance on a surface

		// choose the object which to move depending on the robot travel distance to the location of the respective object
		// TODO: solve traveling salesman problem? Or maybe a bit better (but more time consuming): take pushing pose instead of object's location
		::std::size_t subspacePushObject;
		::rl::math::Real bestDistRobotObject = ::std::numeric_limits< double >::max();
		::rl::math::Real currDistRobotObject;
		const ::rl::math::Transform T = dModel->calcForwardKinematics(startState.segment(0, dModel->getDof(0)), 0);
		for(::std::size_t i=dModel->numRobots; i<dModel->getNumMovableComponents(); ++i)
		{
			if(dModel->isMovingSubspace(startState, endState, i))
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
			::std::cout << "DamaPrimPushMobile::propagate: pushing object nr. " << (subspacePushObject) << " from (" << startState(indexObject) << ", " << startState(indexObject+1) << ", " << startState(indexObject+2) << ") to (" << endState(indexObject) << ", " << endState(indexObject+1) << ", " << endState(indexObject+2) << ")" << std::endl;

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

		// special code for mobile robot
		dirVecMovingObject.normalize();
		dirVecMovingObject *= DamaPrimPush::DIST_TO_OBJECT_XY;

		::rl::math::Vector interConf1(startState);
		interConf1(0) = startState(indexObject) - dirVecMovingObject(0);
		interConf1(1) = startState(indexObject+1) - dirVecMovingObject(1);

		// first move to the pushing position if not already there
		if(dModel->isMovingSubspace(startState, interConf1, 0))
		{
			// TODO: investigate why algorithm currently needs about 5.7% more time just by using the uncommented code instead of the one in comments?!
			//vecResSim.push_back(interConf1);
			//vecEdgeActionSim.push_back(DamaPrimTransit::getInstance()->getName());
			DamaPrimTransit::getInstance()->propagate(vecResSim, interConf1, vecEdgeActionSim);
		}

		// now push the object to its end position
		::rl::math::Vector interConf2(startState);
		interConf2(0) = realEndState(0) - dirVecMovingObject(0);
		interConf2(1) = realEndState(1) - dirVecMovingObject(1);

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

	::rl::math::Transform DamaPrimPushMobile::getToolObjectTransform()
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
