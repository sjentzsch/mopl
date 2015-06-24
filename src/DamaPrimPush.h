/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#ifndef _DAMA_DAMAPRIMPUSH_H_
#define _DAMA_DAMAPRIMPUSH_H_

#include "DamaPrim.h"

namespace dama
{
	class DamaPrimPush : public DamaPrim
	{
	public:
		DamaPrimPush();

		/* distance in meters between the object origin and the robot end-effector in the x-y-plane */
		static rl::math::Real DIST_TO_OBJECT_XY;

		/* offset in meters from the object origin to the robot end-effector in the z-plane */
		static rl::math::Real HEIGHT_OFFSET_OBJECT;

		/* maximum distance in meters the robot is allowed to push an object with a single push operation */
		static rl::math::Real MAX_PUSH_DIST;

		/* maximum joint space distance in radians the robot is allowed to push an object with a single push operation */
		static rl::math::Real MAX_PUSH_DIST_JOINT;


		/* distance in meters that pushing operations will be extended in the x-y-plane [P1 -> P2  =>  P1 -> P3 (new point further away than P2) -> P2] */
		rl::math::Real POSTPROC_EXTEND_DIST;

		rl::math::Real TILT_HAND_INWARD_ANGLE;

		rl::math::Real TILT_HAND_DOWNWARD_ANGLE;

	protected:

	};
}

#endif /* _DAMA_DAMAPRIMPUSH_H_ */
