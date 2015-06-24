/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#ifndef _DAMA_DAMAPRIM_H_
#define _DAMA_DAMAPRIM_H_

#include <rl/math/Vector.h>
#include <rl/sg/Body.h>
#include <rl/sg/so/Body.h>
#include <vector>

namespace dama
{
	class DamaModel;

	class DamaPrim
	{
	public:
		virtual ~DamaPrim() {}

		virtual ::std::string getName() const = 0;
		virtual bool isUseful(const ::rl::math::Vector& startState, const ::rl::math::Vector& endState) = 0;
		virtual bool propagate(::std::vector< ::rl::math::Vector >& vecRes, const ::rl::math::Vector& endState, ::std::vector< ::std::string >& vecEdgeAction) = 0;
		virtual ::rl::math::Transform getToolObjectTransform() = 0;

		::rl::sg::Body* endEffectorBody;
		::rl::sg::so::Body* endEffectorBodyVis;
		DamaModel* dModel;
	};
}

#endif /* _DAMA_DAMAPRIM_H_ */
