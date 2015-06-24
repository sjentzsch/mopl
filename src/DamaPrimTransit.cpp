/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#include "DamaPrimTransit.h"
#include "DamaModel.h"

namespace dama
{
	::std::string DamaPrimTransit::getName() const
	{
		return "Transit";
	}

	bool DamaPrimTransit::isUseful(const ::rl::math::Vector& startState, const ::rl::math::Vector& endState)
	{
		::rl::math::Real dist = 0;
		for(::std::size_t i=this->dModel->numRobots; i<this->dModel->getNumMovableComponents(); ++i)
		{
			if(this->dModel->isMovingSubspace(startState, endState, i))
				return false;

			if(!this->dModel->isOnSupportSurface(startState, i))
				return false;
		}

		return this->dModel->isMovingSubspace(startState, endState, 0);
	}

	bool DamaPrimTransit::propagate(::std::vector< ::rl::math::Vector >& vecRes, const ::rl::math::Vector& endState, ::std::vector< ::std::string >& vecEdgeAction)
	{
		::rl::math::Vector interConf1(vecRes.back());
		for(::std::size_t i=0; i<this->dModel->getDof(0); ++i)
			interConf1(i) = endState(i);
		vecRes.push_back(interConf1);
		vecEdgeAction.push_back(this->getName());

		return true;
	}

	::rl::math::Transform DamaPrimTransit::getToolObjectTransform()
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
