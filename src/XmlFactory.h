/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#ifndef _DAMA_XMLFACTORY_H_
#define _DAMA_XMLFACTORY_H_

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "DamaPrim.h"

namespace dama
{
	class DamaModel;

	class XmlFactory
	{
	public:
		XmlFactory();

		virtual ~XmlFactory();

		boost::shared_ptr< DamaModel > create(const ::std::string& filename, const bool isViewerModel = false);

		void load(const ::std::string& filename, DamaModel* model, const bool isViewerModel = false);

		void setPrimEndEffectorBody(DamaModel* model, ::std::string endEffectorBodySearch, DamaPrim* damaPrim);

	protected:

	private:

	};
}

#endif /* _DAMA_XMLFACTORY_H_ */
