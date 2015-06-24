/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#ifndef _DAMA_DAMAPRIMPUSHEXTERIOR_H_
#define _DAMA_DAMAPRIMPUSHEXTERIOR_H_

#include "DamaPrimPush.h"

namespace dama
{
	class DamaPrimPushExterior : public DamaPrimPush
	{
	public:
		static DamaPrimPushExterior* getInstance()
		{
			static DamaPrimPushExterior instance;
			return &instance;
		}

		virtual ::std::string getName() const;
		virtual bool isUseful(const ::rl::math::Vector& startState, const ::rl::math::Vector& endState);
		virtual bool propagate(::std::vector< ::rl::math::Vector >& vecRes, const ::rl::math::Vector& endState, ::std::vector< ::std::string >& vecEdgeAction);
		virtual ::rl::math::Transform getToolObjectTransform();

	private:
		DamaPrimPushExterior() {};
		DamaPrimPushExterior(DamaPrimPushExterior const&);		// Don't Implement
		void operator=(DamaPrimPushExterior const&);	// Don't implement
	};
}

#endif /* _DAMA_DAMAPRIMPUSHEXTERIOR_H_ */
