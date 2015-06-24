/*
 * Copyright (c) 2015, Sören Jentzsch, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#ifndef _DAMAEXECUTOR_H_
#define _DAMAEXECUTOR_H_

#include <iostream>
#include <list>
#include <fstream>
#include <limits>
#include <string>
#include <Ice/Ice.h>
#include <boost/lexical_cast.hpp>

#include <meka-robot.h>
#include <rl/plan/VectorList.h>
#include "DamaPrim.h"
#include "DamaPrimTransit.h"
#include "DamaPrimPickup.h"
#include "DamaPrimTransferRigid.h"

using namespace std;

class DamaExecutor
{
public:
	static DamaExecutor* getInstance()
	{
		static DamaExecutor instance;
		return &instance;
	}

	::rl::plan::VectorList damaPathVertices;
	::std::vector< ::std::string > damaPathEdges;
	::std::size_t dimVertex;
	::rl::math::Vector maxJointVelocity;	// (in degree/sec)
	::std::size_t numJoints;
	double sampleTime;
	float payloadHandNormal;
	float payloadHandGrasp;

	::rl::plan::VectorList damaPathFinal;
	::std::vector< ::std::string > damaPathFinalAction;

	bool init(::std::string pathFileNameVertices, ::std::string pathFileNameEdges) const;
	bool calcPath(::std::string pathFileName) const;
	bool initRobot() const;
	bool followPathRobot() const;

private:
	DamaExecutor() {};
	DamaExecutor(DamaExecutor const&);		// Don't Implement
	void operator=(DamaExecutor const&);		// Don't implement

	robot::RobotInterfacePrx meka;
};

#endif /* _DAMAEXECUTOR_H_ */
