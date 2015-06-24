/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#include <boost/lexical_cast.hpp>

#include "DamaPrimPush.h"
#include "DamaPrimTransit.h"
#include "DamaModel.h"

namespace dama
{
	DamaPrimPush::DamaPrimPush()
	{
		this->POSTPROC_EXTEND_DIST = 0.1;
		this->TILT_HAND_INWARD_ANGLE = 0.0 * rl::math::DEG2RAD;
		this->TILT_HAND_DOWNWARD_ANGLE = 0.0 * rl::math::DEG2RAD;
	}

	// default values
	rl::math::Real DamaPrimPush::DIST_TO_OBJECT_XY = 0.01;		// important for the xml file: add a small epsilon (here: 0.01), because we need a safety distance
	rl::math::Real DamaPrimPush::HEIGHT_OFFSET_OBJECT = 0.0;
	rl::math::Real DamaPrimPush::MAX_PUSH_DIST = std::numeric_limits<rl::math::Real>::max();
	rl::math::Real DamaPrimPush::MAX_PUSH_DIST_JOINT = std::numeric_limits<rl::math::Real>::max();
}
