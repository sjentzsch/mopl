/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#ifndef _DAMA_DAMAPRIMTRANSIT_H_
#define _DAMA_DAMAPRIMTRANSIT_H_

#include "DamaPrim.h"

namespace dama
{
	class DamaPrimTransit : public DamaPrim
	{
	public:
		static DamaPrimTransit* getInstance()
		{
			static DamaPrimTransit instance;
			return &instance;
		}

		virtual ::std::string getName() const;
		virtual bool isUseful(const ::rl::math::Vector& startState, const ::rl::math::Vector& endState);
		virtual bool propagate(::std::vector< ::rl::math::Vector >& vecRes, const ::rl::math::Vector& endState, ::std::vector< ::std::string >& vecEdgeAction);
		virtual ::rl::math::Transform getToolObjectTransform();

	private:
		DamaPrimTransit() {};
		DamaPrimTransit(DamaPrimTransit const&);	// Don't Implement
		void operator=(DamaPrimTransit const&);		// Don't implement
	};
}

#endif /* _DAMA_DAMAPRIMTRANSIT_H_ */
