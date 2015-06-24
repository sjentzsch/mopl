/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#ifndef _DAMA_DAMARRTCON_H_
#define _DAMA_DAMARRTCON_H_

#include "DamaRrtGoalBias.h"

namespace dama
{
	class DamaRrtCon : public DamaRrtGoalBias
	{
	public:
		DamaRrtCon();

		virtual ~DamaRrtCon();

		virtual ::std::string getName() const;

		bool solve();

	protected:

	private:

	};
}

#endif // _DAMA_DAMARRTCON_H_
