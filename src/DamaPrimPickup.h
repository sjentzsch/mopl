/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#ifndef _DAMA_DAMAPRIMPICKUP_H_
#define _DAMA_DAMAPRIMPICKUP_H_

#include "DamaPrim.h"

namespace dama
{
	class DamaPrimPickup : public DamaPrim
	{
	public:
		static DamaPrimPickup* getInstance()
		{
			static DamaPrimPickup instance;
			return &instance;
		}

		virtual ::std::string getName() const;
		virtual bool isUseful(const ::rl::math::Vector& startState, const ::rl::math::Vector& endState);
		virtual bool propagate(::std::vector< ::rl::math::Vector >& vecRes, const ::rl::math::Vector& endState, ::std::vector< ::std::string >& vecEdgeAction);
		virtual ::rl::math::Transform getToolObjectTransform();

		/* offset in meters from the object origin to the robot end-effector in the z-plane */
		static rl::math::Real HEIGHT_OFFSET_OBJECT;

		/* offset in meters the object will be lifted for pickup in the z-plane */
		static rl::math::Real HEIGHT_PICKUP_DIST_OBJECT;

		/* distance in meters the object will also be picked up if it will be moved within a support surface */
		static rl::math::Real MIN_PLANE_DIST_FOR_PICKUP;

		static rl::math::Real PICKUP_Z_AXIS;
		static rl::math::Real PICKUP_Y_AXIS;
		static rl::math::Real PICKUP_X_AXIS;

	private:
		DamaPrimPickup() {};
		DamaPrimPickup(DamaPrimPickup const&);	// Don't Implement
		void operator=(DamaPrimPickup const&);	// Don't implement
	};
}

#endif /* _DAMA_DAMAPRIMPICKUP_H_ */
