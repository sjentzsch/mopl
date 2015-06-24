/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#ifndef _DAMA_DAMAPRIMPUSHMOBILE_H_
#define _DAMA_DAMAPRIMPUSHMOBILE_H_

#include "DamaPrimPush.h"

namespace dama
{
	class DamaPrimPushMobile : public DamaPrimPush
	{
	public:
		static DamaPrimPushMobile* getInstance()
		{
			static DamaPrimPushMobile instance;
			return &instance;
		}

		virtual ::std::string getName() const;
		virtual bool isUseful(const ::rl::math::Vector& startState, const ::rl::math::Vector& endState);
		virtual bool propagate(::std::vector< ::rl::math::Vector >& vecRes, const ::rl::math::Vector& endState, ::std::vector< ::std::string >& vecEdgeAction);
		virtual ::rl::math::Transform getToolObjectTransform();

	private:
		DamaPrimPushMobile() {};
		DamaPrimPushMobile(DamaPrimPushMobile const&);		// Don't Implement
		void operator=(DamaPrimPushMobile const&);	// Don't implement
	};
}

#endif /* _DAMA_DAMAPRIMPUSHMOBILE_H_ */
