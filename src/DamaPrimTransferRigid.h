/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#ifndef _DAMA_DAMAPRIMTRANSFERRIGID_H_
#define _DAMA_DAMAPRIMTRANSFERRIGID_H_

#include "DamaPrim.h"

namespace dama
{
	class DamaPrimTransferRigid : public DamaPrim
	{
	public:
		static DamaPrimTransferRigid* getInstance()
		{
			static DamaPrimTransferRigid instance;
			return &instance;
		}

		virtual ::std::string getName() const;
		virtual bool isUseful(const ::rl::math::Vector& startState, const ::rl::math::Vector& endState);
		virtual bool propagate(::std::vector< ::rl::math::Vector >& vecRes, const ::rl::math::Vector& endState, ::std::vector< ::std::string >& vecEdgeAction);
		virtual ::rl::math::Transform getToolObjectTransform();

	private:
		DamaPrimTransferRigid() {};
		DamaPrimTransferRigid(DamaPrimTransferRigid const&);		// Don't Implement
		void operator=(DamaPrimTransferRigid const&);				// Don't implement
	};
}

#endif /* _DAMA_DAMAPRIMTRANSFERRIGID_H_ */
