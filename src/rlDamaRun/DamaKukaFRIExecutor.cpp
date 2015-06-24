/*
 * Copyright (c) 2015, Andre Gaschler, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#include "DamaKukaFRIExecutor.h"

const float DamaKukaFRIExecutor::FRI_UPDATE_RATE = 0.002f;

bool DamaKukaFRIExecutor::init(::std::string pathFileNameVertices, ::std::string pathFileNameEdges) const
{
	// 3) determine number of joints
	rl::mdl::XmlFactory factory;
	DamaKukaFRIExecutor::getInstance()->robot_kinematics = dynamic_cast< rl::mdl::Dynamic* >(factory.create("data2/kuka-lwr4/rlmdl/kuka-lwr4-right.xml"));
	DamaKukaFRIExecutor::getInstance()->numJoints = DamaKukaFRIExecutor::getInstance()->robot_kinematics->getDof();

	// 1) read in vertices
	::std::string damaPathSingleVertexLine;
	ifstream pathFileVertices(pathFileNameVertices.c_str());
	if(pathFileVertices.is_open())
	{
		// determine the vertex dimension by counting the commas in the first line and adding one
		if(getline(pathFileVertices,damaPathSingleVertexLine))
		{
			DamaKukaFRIExecutor::getInstance()->dimVertex = std::count(damaPathSingleVertexLine.begin(), damaPathSingleVertexLine.end(), ',') + 1;
			pathFileVertices.seekg(0, ios::beg);
			cout << "Vertex dimension: " << DamaKukaFRIExecutor::getInstance()->dimVertex << endl;
		}
		else
			return false;

		while(getline(pathFileVertices,damaPathSingleVertexLine))
		{
			std::stringstream linestream(damaPathSingleVertexLine);
			std::string	damaPathSingleVertexValue;
			::rl::math::Vector damaPathSingleVertex(DamaKukaFRIExecutor::getInstance()->dimVertex);

			size_t currDim = 0;
			while(getline(linestream, damaPathSingleVertexValue, ','))
			{
				// (left) trim the value string, otherwise lexical cast will fail
				damaPathSingleVertexValue.erase(damaPathSingleVertexValue.begin(), std::find_if(damaPathSingleVertexValue.begin(), damaPathSingleVertexValue.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
				damaPathSingleVertex(currDim) = boost::lexical_cast<double>(damaPathSingleVertexValue);
				currDim++;
			}
			DamaKukaFRIExecutor::getInstance()->damaPathVertices.push_back(damaPathSingleVertex);
		}
		pathFileVertices.close();
	}
	else
		return false;

	// Test output of damaPathVertices
	/*rl::plan::VectorList::iterator i = DamaExecutor::getInstance()->damaPathVertices.begin();
	for(; i != DamaExecutor::getInstance()->damaPathVertices.end(); ++i)
		cout << (*i)(0) << endl;*/

	// 2) read in edges
	::std::string damaPathSingleEdge;
	ifstream pathFileEdges(pathFileNameEdges.c_str());
	if(pathFileEdges.is_open())
	{
		while(getline(pathFileEdges,damaPathSingleEdge))
			DamaKukaFRIExecutor::getInstance()->damaPathEdges.push_back(damaPathSingleEdge);
		pathFileEdges.close();
	}
	else
		return false;

	// 4) determine max. velocities per joint (in degree/sec)
	DamaKukaFRIExecutor::getInstance()->maxJointVelocity.resize(DamaKukaFRIExecutor::getInstance()->numJoints);
	rl::math::Vector speed(DamaKukaFRIExecutor::getInstance()->numJoints);
	DamaKukaFRIExecutor::getInstance()->robot_kinematics->getSpeed(speed);
	for(::std::size_t i=0; i<DamaKukaFRIExecutor::getInstance()->numJoints; ++i)
		DamaKukaFRIExecutor::getInstance()->maxJointVelocity(i) = speed(i) * rl::math::RAD2DEG * SAFETY_LIMITS_MULTIPLIER;

	// 5) determine sample time in seconds
	DamaKukaFRIExecutor::getInstance()->sampleTime = DamaKukaFRIExecutor::FRI_UPDATE_RATE;

	// 6) determine payload values
	//DamaKukaFRIExecutor::getInstance()->payloadHandNormal = 0.9f;
	//DamaKukaFRIExecutor::getInstance()->payloadHandGrasp = 1.6f;

	return true;
}

bool DamaKukaFRIExecutor::calcPath(::std::string pathFileName) const
{
	std::ofstream pathFile;
	pathFile.open(pathFileName.c_str());

	rl::plan::VectorList::iterator v = DamaKukaFRIExecutor::getInstance()->damaPathVertices.begin();
	rl::plan::VectorList::iterator w = ++(DamaKukaFRIExecutor::getInstance()->damaPathVertices.begin());
	::std::size_t segmentCount = 1;
	double executionTime = 0.0;
	for(; v != DamaKukaFRIExecutor::getInstance()->damaPathVertices.end() && w != DamaKukaFRIExecutor::getInstance()->damaPathVertices.end(); ++v, ++w)
	{
		// 1) Determine joint for which the time is limited by the max. velocity -> slowest joint
		::std::size_t slowestJoint = 0;
		double maxTime = 0;
		double currTime;
		for(::std::size_t j=0; j<DamaKukaFRIExecutor::getInstance()->numJoints; ++j)
		{
			currTime = ::std::fabs((*w)(j) - (*v)(j)) / DamaKukaFRIExecutor::getInstance()->maxJointVelocity(j);
			if(currTime > maxTime)
			{
				maxTime = currTime;
				slowestJoint = j;
			}
			//cout << j << ": " << currTime << "ms" << endl;
		}
		executionTime += maxTime;

		cout << "segment " << segmentCount << ": slowestJoint: " << slowestJoint << " | time: " << (maxTime*1000) << "ms" << endl;

		// 2) Calculate the path for the segment using maxTime and sampleTime
		for(double currTime=0; currTime<maxTime; currTime+=DamaKukaFRIExecutor::getInstance()->sampleTime)
		{
			::rl::math::Vector damaPathFinalSingle(DamaKukaFRIExecutor::getInstance()->numJoints);
			DamaKukaFRIExecutor::getInstance()->damaPathFinalAction.push_back(DamaKukaFRIExecutor::getInstance()->damaPathEdges.at(segmentCount-1));
			pathFile << segmentCount << ", " << DamaKukaFRIExecutor::getInstance()->damaPathFinalAction.back() << ", " << (currTime*1000) << "ms, ";
			for(::std::size_t j=0; j<DamaKukaFRIExecutor::getInstance()->numJoints; ++j)
			{
				damaPathFinalSingle(j) = (*v)(j) + (((*w)(j) - (*v)(j)) / maxTime * currTime);
				pathFile << damaPathFinalSingle(j);
				if(j < DamaKukaFRIExecutor::getInstance()->numJoints - 1)
					pathFile << ", ";
			}
			pathFile << endl;
			DamaKukaFRIExecutor::getInstance()->damaPathFinal.push_back(damaPathFinalSingle);
		}

		segmentCount++;
	}
	// manually add the last segment point
	::rl::math::Vector damaPathFinalSingle(DamaKukaFRIExecutor::getInstance()->numJoints);
	DamaKukaFRIExecutor::getInstance()->damaPathFinalAction.push_back(DamaKukaFRIExecutor::getInstance()->damaPathEdges.at(segmentCount-2));
	pathFile << segmentCount << ", " << DamaKukaFRIExecutor::getInstance()->damaPathFinalAction.back() << ", " << 0 << "ms, ";
	for(::std::size_t j=0; j<DamaKukaFRIExecutor::getInstance()->numJoints; ++j)
	{
		damaPathFinalSingle(j) = (*v)(j);
		pathFile << damaPathFinalSingle(j);
		if(j < DamaKukaFRIExecutor::getInstance()->numJoints - 1)
			pathFile << ", ";
	}
	pathFile << endl;
	DamaKukaFRIExecutor::getInstance()->damaPathFinal.push_back(damaPathFinalSingle);

	cout << "overall estimated execution time: " << executionTime << "s" << endl;
	cout << "path elements: " << DamaKukaFRIExecutor::getInstance()->damaPathFinal.size() << endl;

	pathFile.close();

	return true;
}

bool DamaKukaFRIExecutor::initRobot() const
{
#if USE_REAL_ROBOT
	DamaKukaFRIExecutor::getInstance()->robot = new rl::hal::KukaFRI(7, ::std::chrono::duration_cast< ::std::chrono::nanoseconds >(::std::chrono::duration< double >(DamaKukaFRIExecutor::FRI_UPDATE_RATE)), FRI_LIBRARY_DIR "/etc/980039-FRI-Driver.init");
#else
	const char *cmd = "rlmdlcoach data2/kuka-lwr4/rlsg/kuka-lwr4_scene2.xml data2/kuka-lwr4/rlmdl/kuka-lwr4-right.xml &";
	std::cout << "Notice: Running command: " << cmd << std::endl;
	int notused = std::system(cmd);
	int notused2 = std::system("sleep 2");
	DamaKukaFRIExecutor::getInstance()->robot = new rl::hal::Coach(7, ::std::chrono::duration_cast< ::std::chrono::nanoseconds >(::std::chrono::duration< double >(DamaKukaFRIExecutor::FRI_UPDATE_RATE)));
#endif
	{
		struct sched_param param;
		int not_used;
		pthread_getschedparam(pthread_self(), &not_used, &param);
		std::cout << "Debug: Demo pthread_getschedparam: " << param.__sched_priority << std::endl;
	}
	DamaKukaFRIExecutor::getInstance()->gripper = new rl::hal::WeissWsg50();
#if USE_GRIPPER
	DamaKukaFRIExecutor::getInstance()->gripper->open();
	std::cout << "Notice: Starting gripper." << std::endl;
	DamaKukaFRIExecutor::getInstance()->gripper->start();
	DamaKukaFRIExecutor::getInstance()->gripper->doAcknowledgeFaults();
	DamaKukaFRIExecutor::getInstance()->gripper->doSetAcceleration(0.2f);
	DamaKukaFRIExecutor::getInstance()->gripper->doSetForceLimit(20.0f);
	std::cout << "Notice: Gripper is running." << std::endl;
#endif
	DamaKukaFRIExecutor::getInstance()->robot->open();
	std::cout << "Notice: Starting robot." << std::endl;
	DamaKukaFRIExecutor::getInstance()->robot->start();
#if USE_REAL_ROBOT
	//DamaKukaFRIExecutor::getInstance()->robot->getJointPosition(robot_q_current);
	DamaKukaFRIExecutor::getInstance()->robot->step(); // Workaround: Init position control, then impedance
	DamaKukaFRIExecutor::getInstance()->robot->stop();
	DamaKukaFRIExecutor::getInstance()->robot->start(FastResearchInterface::JOINT_IMPEDANCE_CONTROL);
	DamaKukaFRIExecutor::getInstance()->robot->step();
#endif
	std::cout << "Notice: Robot is running." << std::endl;

	return true;
}


bool DamaKukaFRIExecutor::shutdownRobot() const
{
#if USE_GRIPPER
	DamaKukaFRIExecutor::getInstance()->gripper->stop();
	DamaKukaFRIExecutor::getInstance()->gripper->close();
#endif
	std::cout << "Notice: Done." << std::endl;
	DamaKukaFRIExecutor::getInstance()->robot->stop();
#if USE_REAL_ROBOT
	DamaKukaFRIExecutor::getInstance()->robot->start(); // Workaround: Reset to position control
	DamaKukaFRIExecutor::getInstance()->robot->step();
	DamaKukaFRIExecutor::getInstance()->robot->stop();
#endif
	return true;
}

bool DamaKukaFRIExecutor::followPathRobot() const
{
	::std::string pickupPrefix = "Pickup ";
	::std::string transferPrefix = "Transfer-Rigid ";
	::std::string pushPrefix = "Push";

	rl::math::Vector robot_q_current(DamaKukaFRIExecutor::getInstance()->robot->getDof());
#if USE_REAL_ROBOT
	DamaKukaFRIExecutor::getInstance()->robot->getJointPosition(robot_q_current);
#else
	robot_q_current = 5.0f * ::rl::math::Vector::Ones(robot_q_current.size()) * ::rl::math::DEG2RAD;
#endif
	rl::plan::VectorList::iterator v = DamaKukaFRIExecutor::getInstance()->damaPathFinal.begin();
	rl::math::Vector robot_q_first = *v * rl::math::DEG2RAD;
	cout << "Driving from current position to first node." << endl; cout.flush();
	for(rl::math::Real a = 0; a < 1; a += ::std::chrono::duration_cast< ::std::chrono::duration< double > >(DamaKukaFRIExecutor::getInstance()->robot->getUpdateRate()).count() / 1.5f)
	{
		rl::math::Vector robot_q = (1-a) * robot_q_current + (a * robot_q_first);
		DamaKukaFRIExecutor::getInstance()->robot->setJointPosition(robot_q);
		DamaKukaFRIExecutor::getInstance()->robot->step();
	}

	::std::vector< ::std::string >::iterator a = DamaKukaFRIExecutor::getInstance()->damaPathFinalAction.begin();
	::std::string lastAction = "NULL";
	for(; v != DamaKukaFRIExecutor::getInstance()->damaPathFinal.end() && a != DamaKukaFRIExecutor::getInstance()->damaPathFinalAction.end(); ++v, ++a)
	{
		// TODO: set stiffness and slewRate according to action type?
		//if((*a) == "Transit") ...

		if((a->substr(0, transferPrefix.size()) != transferPrefix) && (lastAction.substr(0, transferPrefix.size()) == transferPrefix))
		{
			cout << "do hand release ... ";
			cout.flush();

			// decrease payload incrementally
			/* float payloadHandCurr = DamaKukaFRIExecutor::getInstance()->payloadHandGrasp;
			while(payloadHandCurr - 0.1f >= DamaKukaFRIExecutor::getInstance()->payloadHandNormal - 1.0e-3f)
			{
				payloadHandCurr -= 0.1f;
				//DamaExecutor::getInstance()->meka->setHandPayload(Ice::Float(payloadHandCurr));
				usleep(0.1 * 1000 * 1000);
			} */
#if USE_GRIPPER
			try{
				gripper->doPrePositionFingers(0.10);
			} catch(rl::hal::Exception& e){
				std::cout << "Warning: rl::hal::WeissWsg50::Exception " << e.what() << std::endl;
			}
#endif
			dama::Timer::sleep(0.01); // avoid jerk
			//DamaKukaFRIExecutor::getInstance()->meka->doHandRelease();
			//usleep(1.0 * 1000 * 1000);
			//DamaKukaFRIExecutor::getInstance()->meka->doHandPushPos();
			//usleep(2.7 * 1000 * 1000);

			cout << "done." << endl;
		}

		if((a->substr(0, pickupPrefix.size()) == pickupPrefix) && (lastAction.substr(0, pickupPrefix.size()) != pickupPrefix))
		{
			/*cout << "hand grasp: " << *a << endl;
			cout << *v << endl;*/

			cout << "do hand grasp ... ";
			cout.flush();

#if USE_GRIPPER
			try{
				DamaKukaFRIExecutor::getInstance()->gripper->doPrePositionFingers(0.01);
			} catch(rl::hal::Exception& e){
				std::cout << "Warning: rl::hal::WeissWsg50::Exception " << e.what() << std::endl;
			}
#endif
			dama::Timer::sleep(0.5); // avoid jerk
			//DamaExecutor::getInstance()->meka->doHandRelease();
			//usleep(2.0 * 1000 * 1000);
			//DamaExecutor::getInstance()->meka->doHandGrasp();
			//usleep(2.5 * 1000 * 1000);

			// increase payload incrementally
			/* float payloadHandCurr = DamaKukaFRIExecutor::getInstance()->payloadHandNormal;
			while(payloadHandCurr + 0.1f <= DamaKukaFRIExecutor::getInstance()->payloadHandGrasp + 1.0e-3f)
			{
				payloadHandCurr += 0.1f;
				//DamaExecutor::getInstance()->meka->setHandPayload(Ice::Float(payloadHandCurr));
				usleep(0.1 * 1000 * 1000);
			} */

			cout << "done." << endl;
		}

		/*
		robot::SequenceReal qTorso;
		for(::std::size_t j=0; j<3; ++j)
			qTorso.push_back((::Ice::Float)(*v)(j));

		robot::SequenceReal qArm;
		for(::std::size_t j=3; j<10; ++j)
			qArm.push_back((::Ice::Float)(*v)(j));

		Ice::AsyncResultPtr r1 = DamaExecutor::getInstance()->meka->begin_setThetaDeg("torso", qTorso);
		Ice::AsyncResultPtr r2 = DamaExecutor::getInstance()->meka->begin_setThetaDeg("right_arm", qArm);

		DamaExecutor::getInstance()->meka->end_setThetaDeg(r1);
		DamaExecutor::getInstance()->meka->end_setThetaDeg(r2);
		DamaExecutor::getInstance()->meka->proxyStep();
		*/

		dama::Timer timer;
		DamaKukaFRIExecutor::getInstance()->robot->setJointPosition(*v * rl::math::DEG2RAD);
		// std::cout << "Debug: *v * rl::math::DEG2RAD " << (*v).transpose() * rl::math::DEG2RAD << std::endl;
		timer.start();
		DamaKukaFRIExecutor::getInstance()->robot->step();
		timer.stop();
		if(timer.elapsedDuration() < DamaKukaFRIExecutor::getInstance()->robot->getUpdateRate() * 0.75)
			printf("Warning: Real-time not maintained, Robot::step took only %1.7f s during trajectory.\n", timer.elapsed());
#if !USE_REAL_ROBOT
		//rl::util::Timer::sleep(DamaKukaFRIExecutor::FRI_UPDATE_RATE);
#endif

		if(v == DamaKukaFRIExecutor::getInstance()->damaPathFinal.begin())
		{
			// sleep 5 seconds long due to driving to starting position
			cout << "driving to starting position 1 seconds long ... ";
			cout.flush();
#if USE_GRIPPER
			try{
				gripper->doPrePositionFingers(0.10);
			} catch(rl::hal::Exception& e){
				std::cout << "Warning: rl::hal::WeissWsg50::Exception " << e.what() << std::endl;
			}
#endif
			dama::Timer::sleep(1.0);
			cout << "done." << endl;
		}
		else if((a+1) != DamaKukaFRIExecutor::getInstance()->damaPathFinalAction.end() && (*a) != (*(a+1)))
		{
			cout << "mode change from " << (*a) << " to " << *(a+1) << ", wait 0.2 seconds long ... ";
			cout.flush();
			dama::Timer::sleep(0.2f);
			cout << "done." << endl;

			if((a+1)->substr(0, pushPrefix.size()) == pushPrefix)
			{
				cout << "push pause, wait 1.0 seconds long ... ";
				cout.flush();
				dama::Timer::sleep(1.0f);
				cout << "done." << endl;
			}
		}
		else
		{
			// sleep sampleTime long
			//usleep(DamaExecutor::getInstance()->sampleTime * 1000 * 1000);
		}

		lastAction = *a;
	}

	cout << "finishing, wait 1.0 seconds long ... ";
	cout.flush();
	dama::Timer::sleep(1.0f);

	return true;
}
