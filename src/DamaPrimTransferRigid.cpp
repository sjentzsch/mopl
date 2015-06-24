/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#include <boost/lexical_cast.hpp>

#include "DamaPrimTransferRigid.h"
#include "DamaModel.h"

namespace dama
{
	::std::string DamaPrimTransferRigid::getName() const
	{
		return "Transfer-Rigid Object";
	}

	bool DamaPrimTransferRigid::isUseful(const ::rl::math::Vector& startState, const ::rl::math::Vector& endState)
	{
		for(::std::size_t i=dModel->numRobots; i<dModel->getNumMovableComponents(); ++i)
		{
			::std::size_t offset = this->dModel->indexFromSubspace(i);
			//::rl::math::Real distPlaneTransformed = ::std::pow(startState(offset+0) - endState(offset+0), 2) + ::std::pow(startState(offset+1) - endState(offset+1), 2);
			//::rl::math::Real distHeight = ::std::fabs(startState(offset+2) - endState(offset+2));
			::rl::math::Real distEuclidTransformed = ::std::pow(startState(offset+0) - endState(offset+0), 2) + ::std::pow(startState(offset+1) - endState(offset+1), 2) + ::std::pow(startState(offset+2) - endState(offset+2), 2);

			// if the object is in the air, and either the object has to move or we change the grasp
			if(!this->dModel->isOnSupportSurface(startState, i) && (distEuclidTransformed > dModel->epsilonTransformed || dModel->isMovingSubspace(startState, endState, 0)))
				return true;
		}

		return false;
	}

	bool DamaPrimTransferRigid::propagate(::std::vector< ::rl::math::Vector >& vecRes, const ::rl::math::Vector& endState, ::std::vector< ::std::string >& vecEdgeAction)
	{
		dama::Timer timerIK;

		::std::vector< ::rl::math::Vector > vecResSim = vecRes;
		::std::vector< ::std::string > vecEdgeActionSim = vecEdgeAction;
		::rl::math::Vector startState = vecResSim.back();

		// choose the object which to move depending on the robot travel distance to the location of the respective object
		// TODO: solve traveling salesman problem? Or maybe a bit better (but more time consuming): take pushing pose instead of object's location
		// TODO: optimize: there should be only one object which could possibly "fly in the air"
		::std::size_t subspacePushObject;
		::rl::math::Real bestDistRobotObject = ::std::numeric_limits< double >::max();
		::rl::math::Real currDistRobotObject;
		const ::rl::math::Transform T = dModel->calcForwardKinematics(startState.segment(0, dModel->getDof(0)), 0, this->dModel->mdlGrasp);
		for(::std::size_t i=dModel->numRobots; i<dModel->getNumMovableComponents(); ++i)
		{
			::std::size_t offset = this->dModel->indexFromSubspace(i);
			//::rl::math::Real distPlane = ::std::fabs(startState(offset+0) - endState(offset+0)) + ::std::fabs(startState(offset+1) - endState(offset+1));
			//::rl::math::Real distHeight = ::std::fabs(startState(offset+2) - endState(offset+2));
			::rl::math::Real distEuclidTransformed = ::std::pow(startState(offset+0) - endState(offset+0), 2) + ::std::pow(startState(offset+1) - endState(offset+1), 2) + ::std::pow(startState(offset+2) - endState(offset+2), 2);

			if(!this->dModel->isOnSupportSurface(startState, i) && (distEuclidTransformed > dModel->epsilonTransformed || dModel->isMovingSubspace(startState, endState, 0)))
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
			::std::cout << "DamaPrimTransferRigid::propagate: transfer object nr. " << (subspacePushObject) << " from (" << startState(indexObject) << ", " << startState(indexObject+1) << ", " << startState(indexObject+2) << ") to (" << endState(indexObject) << ", " << endState(indexObject+1) << ", " << endState(indexObject+2) << ")" << std::endl;

		//TODO: maybe we don't need the IK anymore, because sample comes with right robot arm pose already now?!

		::rl::math::Vector interConf2(startState);
		if(this->dModel->isOnSupportSurface(endState, subspacePushObject))
		{
			// calculate the robot transfer end position
			rl::math::Transform T2_transfer;
			T2_transfer.translation().x() = endState(indexObject);
			T2_transfer.translation().y() = endState(indexObject+1);
			T2_transfer.translation().z() = endState(indexObject+2) + DamaPrimPickup::HEIGHT_OFFSET_OBJECT;
			T2_transfer.linear() = (
				rl::math::AngleAxis(DamaPrimPickup::PICKUP_Z_AXIS, rl::math::Vector3::UnitZ()) *	// WILL BE FREE AXIS
				rl::math::AngleAxis(DamaPrimPickup::PICKUP_Y_AXIS, rl::math::Vector3::UnitY()) *
				rl::math::AngleAxis(DamaPrimPickup::PICKUP_X_AXIS, rl::math::Vector3::UnitX())
			).toRotationMatrix();

			rl::math::Matrix66 constraint_position_orient = rl::math::Matrix66::Zero();
			constraint_position_orient.diagonal() << 1, 1, 1, 1, 1, 0;

			rl::math::Vector q_ik_start2 = startState.segment(0, dModel->getDof(0));

			if(this->dModel->debugMode)
			{
				::std::cout << "Calculate IK2 to T = (";
				this->dModel->printTransform(T2_transfer, false);
				::std::cout << ") starting at q_deg = (";
				this->dModel->printQLine(q_ik_start2 * rl::math::RAD2DEG, false, false, false, false);
				::std::cout << ")" << ::std::endl;
			}

			timerIK.start();
			bool calcInverse2 = DamaModel::calcInversePositionTaskPosture(dModel->mdlGrasp, T2_transfer, q_ik_start2, constraint_position_orient, this->dModel->coupleJoint1And2);
			timerIK.stop();
			this->dModel->timeIK += timerIK.elapsed();

			if(!calcInverse2)
			{
				if(this->dModel->debugMode)
					::std::cout << "Result: IK2 failed. Exit." << ::std::endl;
				return false;
			}

			this->dModel->updateRobotPosition(interConf2, this->dModel->mdlGrasp);
		}
		else
		{
			if(this->dModel->debugMode)
				::std::cout << "Calculate IK2 not needed, just take the pose from the sample, resp. end-state" << ::std::endl;
			for(::std::size_t i=0; i<this->dModel->getDof(0); ++i)
				interConf2(i) = endState(i);
		}

		if(this->dModel->debugMode)
		{
			::std::cout << "Result: IK2 succeeded with q_deg = (";
			this->dModel->printQLine(interConf2, false, false, false, true);
			::std::cout << ")" << ::std::endl;
		}

		interConf2(indexObject) = endState(indexObject+0);
		interConf2(indexObject+1) = endState(indexObject+1);
		interConf2(indexObject+2) = endState(indexObject+2);

		vecResSim.push_back(interConf2);
		vecEdgeActionSim.push_back(this->getName() + " " + boost::lexical_cast<std::string>(subspacePushObject));

		vecRes = vecResSim;
		vecEdgeAction = vecEdgeActionSim;

		return true;
	}

	::rl::math::Transform DamaPrimTransferRigid::getToolObjectTransform()
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
