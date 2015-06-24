/*
 * Copyright (c) 2015, Andre Gaschler, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#ifndef _DAMA_KUKA_FRI_EXECUTOR_H_
#define _DAMA_KUKA_FRI_EXECUTOR_H_

#include <iostream>
#include <list>
#include <fstream>
#include <limits>
#include <string>
#include <boost/lexical_cast.hpp>

#include <rl/plan/VectorList.h>
#include "DamaPrim.h"
#include "DamaPrimTransit.h"
#include "DamaPrimPickup.h"
#include "DamaPrimTransferRigid.h"
#include "Timer.h"

#include <rl/hal/Coach.h>
#include <rl/hal/ComException.h>
#include <rl/hal/WeissWsg50.h>
#if USE_REAL_ROBOT
#include "KukaFRI.h"
#endif
#include <rl/mdl/Dynamic.h>
#include <rl/mdl/XmlFactory.h>

using namespace std;

// 1==real limits  0.5==half speed
#define SAFETY_LIMITS_MULTIPLIER 0.15

#if USE_REAL_ROBOT
# define USE_GRIPPER 1
#else
# define USE_GRIPPER 0
#endif

class DamaKukaFRIExecutor
{
public:
	static DamaKukaFRIExecutor* getInstance()
	{
		static DamaKukaFRIExecutor instance;
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
	bool shutdownRobot() const;

	static const float FRI_UPDATE_RATE;

private:
	DamaKukaFRIExecutor() {};
	~DamaKukaFRIExecutor(){
		delete robot;
		delete gripper;
		//delete robot_kinematics;
	}
	DamaKukaFRIExecutor(DamaKukaFRIExecutor const&);		// Don't Implement
	void operator=(DamaKukaFRIExecutor const&);		// Don't implement

#if USE_REAL_ROBOT
	rl::hal::KukaFRI*
#else
	rl::hal::Coach*
#endif
	robot;

	rl::hal::WeissWsg50* gripper;
	rl::mdl::Dynamic* robot_kinematics;
};

#endif /* _DAMA_KUKA_FRI_EXECUTOR_H_ */
