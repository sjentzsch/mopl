/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#ifndef _DAMA_DAMARRTGOALBIAS_H_
#define _DAMA_DAMARRTGOALBIAS_H_

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>

#include "DamaRrtAction.h"

namespace dama
{
	class DamaRrtGoalBias : public DamaRrtAction
	{
	public:
		DamaRrtGoalBias();

		virtual ~DamaRrtGoalBias();

		virtual ::std::string getName() const;

		virtual void seed(const ::boost::mt19937::result_type& value);

		/** Probability of choosing goal configuration. */
		::rl::math::Real probability;

	protected:
		virtual void choose(::rl::math::Vector& chosen);

		::boost::variate_generator< ::boost::mt19937, ::boost::uniform_real< ::rl::math::Real > > rand;

	private:

	};
}

#endif // _DAMA_DAMARRTGOALBIAS_H_
