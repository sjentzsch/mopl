/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#ifndef _DAMA_DAMAPRIMPUSHINTERIOR_H_
#define _DAMA_DAMAPRIMPUSHINTERIOR_H_

#include "DamaPrimPush.h"

namespace dama
{
	class DamaPrimPushInterior : public DamaPrimPush
	{
	public:
		static DamaPrimPushInterior* getInstance()
		{
			static DamaPrimPushInterior instance;
			return &instance;
		}

		virtual ::std::string getName() const;
		virtual bool isUseful(const ::rl::math::Vector& startState, const ::rl::math::Vector& endState);
		virtual bool propagate(::std::vector< ::rl::math::Vector >& vecRes, const ::rl::math::Vector& endState, ::std::vector< ::std::string >& vecEdgeAction);
		virtual ::rl::math::Transform getToolObjectTransform();

	private:
		DamaPrimPushInterior() {};
		DamaPrimPushInterior(DamaPrimPushInterior const&);		// Don't Implement
		void operator=(DamaPrimPushInterior const&);	// Don't implement
	};
}

#endif /* _DAMA_DAMAPRIMPUSHINTERIOR_H_ */
