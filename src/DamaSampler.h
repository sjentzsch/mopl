/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#ifndef _DAMA_DAMASAMPLER_H_
#define _DAMA_DAMASAMPLER_H_

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <vector>

#include <rl/plan/Sampler.h>

namespace dama
{
	class DamaModel;

	class DamaSampler : public ::rl::plan::Sampler
	{
	public:
		DamaSampler();

		virtual ~DamaSampler();

		virtual void init();

		virtual void generate(::rl::math::Vector& q);

		virtual void generateOnly(::rl::math::Vector& q, const ::std::size_t subspace, const ::std::size_t supportSurface, const ::std::vector<bool>* qIsDefined = NULL);

		virtual void generateOnlyCollisionFree(::rl::math::Vector& q, const ::std::size_t subspace, const ::std::size_t supportSurface, const ::std::vector<bool>* qIsDefined = NULL);

		virtual void seed(const ::boost::mt19937::result_type& value);

		DamaModel* dModel;

	protected:
		::boost::variate_generator< ::boost::mt19937, ::boost::uniform_real< ::rl::math::Real > > randReal;
		::std::vector< ::rl::math::Vector > vecMaximum;
		::std::vector< ::rl::math::Vector > vecMinimum;

	private:

	};
}

#endif /* _DAMA_DAMASAMPLER_H_ */
