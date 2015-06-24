/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#ifndef _DAMA_DAMAPRIMPUSHFRONTAL_H_
#define _DAMA_DAMAPRIMPUSHFRONTAL_H_

#include "DamaPrimPush.h"

namespace dama
{
	class DamaPrimPushFrontal : public DamaPrimPush
	{
	public:
		static DamaPrimPushFrontal* getInstance()
		{
			static DamaPrimPushFrontal instance;
			return &instance;
		}

		virtual ::std::string getName() const;
		virtual bool isUseful(const ::rl::math::Vector& startState, const ::rl::math::Vector& endState);
		virtual bool propagate(::std::vector< ::rl::math::Vector >& vecRes, const ::rl::math::Vector& endState, ::std::vector< ::std::string >& vecEdgeAction);
		virtual ::rl::math::Transform getToolObjectTransform();

	private:
		DamaPrimPushFrontal() {};
		DamaPrimPushFrontal(DamaPrimPushFrontal const&);		// Don't Implement
		void operator=(DamaPrimPushFrontal const&);	// Don't implement
	};
}

#endif /* _DAMA_DAMAPRIMPUSHFRONTAL_H_ */
