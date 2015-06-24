/*
 * Copyright (c) 2015, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#ifndef DAMAPLANNER_H_
#define DAMAPLANNER_H_

#include <rl/math/Vector.h>

#include "DamaModel.h"

class DamaPlanner
{
public:
	static DamaPlanner* getInstance()
	{
		static DamaPlanner instance;
		return &instance;
	}

	bool init(::std::string newTaskFileName);

	bool run();

	boost::shared_ptr< dama::DamaModel > dModel;
	boost::shared_ptr< dama::DamaModel > dModelVis;

	boost::shared_ptr< rl::math::Vector > startInit;
	boost::shared_ptr< rl::math::Vector > goalInit;
	boost::shared_ptr< ::std::vector<bool> > goalDimDefinedInit;

	::std::vector< bool > vecSolved;
	::std::vector< rl::plan::VectorList > vecPath;
	::std::vector< ::std::deque< ::std::string > > vecActions;

	::std::string getTaskFileName() const;

private:
	DamaPlanner() {};
	DamaPlanner(DamaPlanner const&);	// Don't Implement
	void operator=(DamaPlanner const&);		// Don't implement

	template <typename T> ::rl::math::Real getAvgValue(::std::vector< T > vecValue, bool considerOnlySolvedRuns) const;

	::std::string taskFileName;
};

#endif /* DAMAPLANNER_H_ */
