/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#include <rl/plan/Sampler.h>

#include "DamaRrtGoalBias.h"
#include "Timer.h"

namespace dama
{
	DamaRrtGoalBias::DamaRrtGoalBias() :
		DamaRrtAction(),
		probability(0.05f),
		rand(
			::boost::mt19937(static_cast< ::boost::mt19937::result_type >(0)),
			::boost::uniform_real< ::rl::math::Real >(0.0f, 1.0f)
		)
	{
	}

	DamaRrtGoalBias::~DamaRrtGoalBias()
	{
	}

	void
	DamaRrtGoalBias::choose(::rl::math::Vector& chosen)
	{
		if (this->rand() > this->probability)
		{
			DamaRrtAction::choose(chosen);
		}
		else
		{
			chosen = *this->goal;
		}
	}

	::std::string
	 DamaRrtGoalBias::getName() const
	{
		return "RRT Goal Bias";
	}

	void
	DamaRrtGoalBias::seed(const ::boost::mt19937::result_type& value)
	{
		this->rand.engine().seed(value);
	}
}
