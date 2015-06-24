/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#include <boost/lexical_cast.hpp>

#include "DamaSupportSurface.h"
#include "DamaModel.h"

namespace dama
{
	DamaSupportSurface::DamaSupportSurface(::std::string _name, ::rl::math::Real _height, const ::rl::math::Vector& _min, const ::rl::math::Vector& _max)
	{
		this->name = _name;
		this->height = _height;
		this->dim = _max.size();
		this->min = _min;
		this->max = _max;
	}

	::std::string DamaSupportSurface::getName()
	{
		return this->name;
	}

	::rl::math::Real DamaSupportSurface::getHeight()
	{
		return this->height;
	}

	::rl::math::Vector DamaSupportSurface::getMin()
	{
		return this->min;
	}

	::rl::math::Real DamaSupportSurface::getMin(::std::size_t i)
	{
		return (this->min)(i);
	}

	::rl::math::Vector DamaSupportSurface::getMax()
	{
		return this->max;
	}

	::rl::math::Real DamaSupportSurface::getMax(::std::size_t i)
	{
		return (this->max)(i);
	}
}
