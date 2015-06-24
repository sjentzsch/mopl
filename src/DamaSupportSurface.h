/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#ifndef _DAMA_DAMASUPPORTSURFACE_H_
#define _DAMA_DAMASUPPORTSURFACE_H_

#include <rl/math/Vector.h>
#include <vector>
#include <string>

namespace dama
{
	class DamaSupportSurface
	{
	public:
		DamaSupportSurface(::std::string _name, ::rl::math::Real _height, const ::rl::math::Vector& _min, const ::rl::math::Vector& _max);
		~DamaSupportSurface() {};

		::std::string getName();
		::rl::math::Real getHeight();

		::rl::math::Vector getMin();
		::rl::math::Real getMin(::std::size_t i);
		::rl::math::Vector getMax();
		::rl::math::Real getMax(::std::size_t i);

	private:
		::std::string name;
		::rl::math::Real height;
		::std::size_t dim;
		::rl::math::Vector min;
		::rl::math::Vector max;
	};
}

#endif /* _DAMA_DAMASUPPORTSURFACE_H_ */
