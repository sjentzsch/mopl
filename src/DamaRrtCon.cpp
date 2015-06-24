/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#include "DamaRrtCon.h"
#include <rl/plan/SimpleModel.h>

namespace dama
{
	DamaRrtCon::DamaRrtCon() :
		DamaRrtGoalBias()
	{
	}

	DamaRrtCon::~DamaRrtCon()
	{
	}

	::std::string
	 DamaRrtCon::getName() const
	{
		return "RRT Connect";
	}

	bool
	DamaRrtCon::solve()
	{
		this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared< ::rl::math::Vector >(*this->start));
		
		::rl::math::Vector chosen(this->model->getDof());
		
		timer.start();
		timer.stop();
		
		while (timer.elapsedDuration() < this->duration)
		{
			this->choose(chosen);
			
			Neighbor nearest = this->nearest(this->tree[0], chosen);
			
			Vertex connected = this->connect(this->tree[0], nearest, chosen);
			
			if (NULL != connected)
			{
				if (this->areEqual(*this->tree[0][connected].q, *this->goal))
				{
					this->end[0] = connected;
					return true;
				}
			}
			
			timer.stop();
		}

		return false;
	}
}
