/*
 * Copyright (c) 2015, Sören Jentzsch, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#include "DamaExecutor.h"

bool DamaExecutor::init(::std::string pathFileNameVertices, ::std::string pathFileNameEdges) const
{
	// 1) read in vertices
	::std::string damaPathSingleVertexLine;
	ifstream pathFileVertices(pathFileNameVertices.c_str());
	if(pathFileVertices.is_open())
	{
		// determine the vertex dimension by counting the commas in the first line and adding one
		if(getline(pathFileVertices,damaPathSingleVertexLine))
		{
			DamaExecutor::getInstance()->dimVertex = std::count(damaPathSingleVertexLine.begin(), damaPathSingleVertexLine.end(), ',') + 1;
			pathFileVertices.seekg(0, ios::beg);
			cout << "Vertex dimension: " << DamaExecutor::getInstance()->dimVertex << endl;
		}
		else
			return false;

		while(getline(pathFileVertices,damaPathSingleVertexLine))
		{
			std::stringstream linestream(damaPathSingleVertexLine);
			std::string	damaPathSingleVertexValue;
			::rl::math::Vector damaPathSingleVertex(DamaExecutor::getInstance()->dimVertex);

			size_t currDim = 0;
			while(getline(linestream, damaPathSingleVertexValue, ','))
			{
				// (left) trim the value string, otherwise lexical cast will fail
				damaPathSingleVertexValue.erase(damaPathSingleVertexValue.begin(), std::find_if(damaPathSingleVertexValue.begin(), damaPathSingleVertexValue.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
				damaPathSingleVertex(currDim) = boost::lexical_cast<double>(damaPathSingleVertexValue);
				currDim++;
			}
			DamaExecutor::getInstance()->damaPathVertices.push_back(damaPathSingleVertex);
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
			DamaExecutor::getInstance()->damaPathEdges.push_back(damaPathSingleEdge);
		pathFileEdges.close();
	}
	else
		return false;

	// 3) determine number of joints
	DamaExecutor::getInstance()->numJoints = 10.0;

	// 4) determine max. velocities per joint (in degree/sec)
	DamaExecutor::getInstance()->maxJointVelocity.resize(DamaExecutor::getInstance()->numJoints);
	DamaExecutor::getInstance()->maxJointVelocity(0) = 20.0;	// 30.0
	DamaExecutor::getInstance()->maxJointVelocity(1) = 17.0;	// 50.0		10.0
	DamaExecutor::getInstance()->maxJointVelocity(2) = 17.0;	// 50.0		10.0
	for(::std::size_t i=3; i<DamaExecutor::getInstance()->numJoints; ++i)
		DamaExecutor::getInstance()->maxJointVelocity(i) = 40.0;	// 80.0

	// 5) determine sample time in seconds
	DamaExecutor::getInstance()->sampleTime = 0.01;	// 0.02

	// 6) determine payload values
	DamaExecutor::getInstance()->payloadHandNormal = 0.9f;
	DamaExecutor::getInstance()->payloadHandGrasp = 1.6f;

	return true;
}

bool DamaExecutor::calcPath(::std::string pathFileName) const
{
	std::ofstream pathFile;
	pathFile.open(pathFileName.c_str());

	rl::plan::VectorList::iterator v = DamaExecutor::getInstance()->damaPathVertices.begin();
	rl::plan::VectorList::iterator w = ++(DamaExecutor::getInstance()->damaPathVertices.begin());
	::std::size_t segmentCount = 1;
	double executionTime = 0.0;
	for(; v != DamaExecutor::getInstance()->damaPathVertices.end() && w != DamaExecutor::getInstance()->damaPathVertices.end(); ++v, ++w)
	{
		// 1) Determine joint for which the time is limited by the max. velocity -> slowest joint
		::std::size_t slowestJoint = 0;
		double maxTime = 0;
		double currTime;
		for(::std::size_t j=0; j<DamaExecutor::getInstance()->numJoints; ++j)
		{
			currTime = ::std::fabs((*w)(j) - (*v)(j)) / DamaExecutor::getInstance()->maxJointVelocity(j);
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
		for(double currTime=0; currTime<maxTime; currTime+=DamaExecutor::getInstance()->sampleTime)
		{
			::rl::math::Vector damaPathFinalSingle(DamaExecutor::getInstance()->numJoints);
			DamaExecutor::getInstance()->damaPathFinalAction.push_back(DamaExecutor::getInstance()->damaPathEdges.at(segmentCount-1));
			pathFile << segmentCount << ", " << DamaExecutor::getInstance()->damaPathFinalAction.back() << ", " << (currTime*1000) << "ms, ";
			for(::std::size_t j=0; j<DamaExecutor::getInstance()->numJoints; ++j)
			{
				damaPathFinalSingle(j) = (*v)(j) + (((*w)(j) - (*v)(j)) / maxTime * currTime);
				pathFile << damaPathFinalSingle(j);
				if(j < DamaExecutor::getInstance()->numJoints - 1)
					pathFile << ", ";
			}
			pathFile << endl;
			DamaExecutor::getInstance()->damaPathFinal.push_back(damaPathFinalSingle);
		}

		segmentCount++;
	}
	// manually add the last segment point
	::rl::math::Vector damaPathFinalSingle(DamaExecutor::getInstance()->numJoints);
	DamaExecutor::getInstance()->damaPathFinalAction.push_back(DamaExecutor::getInstance()->damaPathEdges.at(segmentCount-2));
	pathFile << segmentCount << ", " << DamaExecutor::getInstance()->damaPathFinalAction.back() << ", " << 0 << "ms, ";
	for(::std::size_t j=0; j<DamaExecutor::getInstance()->numJoints; ++j)
	{
		damaPathFinalSingle(j) = (*v)(j);
		pathFile << damaPathFinalSingle(j);
		if(j < DamaExecutor::getInstance()->numJoints - 1)
			pathFile << ", ";
	}
	pathFile << endl;
	DamaExecutor::getInstance()->damaPathFinal.push_back(damaPathFinalSingle);

	cout << "overall estimated execution time: " << executionTime << "s" << endl;
	cout << "path elements: " << DamaExecutor::getInstance()->damaPathFinal.size() << endl;

	pathFile.close();

	return true;
}

bool DamaExecutor::initRobot() const
{
	Ice::CommunicatorPtr ic;
	try {
		// Ice Init
		ic = Ice::initialize();
		Ice::ObjectPrx base = ic->stringToProxy("Robot:default:tcp -h 192.168.21.52 -p 5001");
		DamaExecutor::getInstance()->meka = robot::RobotInterfacePrx::checkedCast(base);
		if (!DamaExecutor::getInstance()->meka)  throw "Invalid proxy";

		// Set ModeThetaGc
		DamaExecutor::getInstance()->meka->setModeThetaGc("right_arm");
		DamaExecutor::getInstance()->meka->setModeThetaGc("torso");

		// Set Stiffness and SlewRate of the Arm
		robot::SequenceReal stiffnessArm = robot::SequenceReal();
		robot::SequenceReal slewRateArm = robot::SequenceReal();
		for(int i=0; i<7; ++i)
		{
			stiffnessArm.push_back(Ice::Float(0.9));
			slewRateArm.push_back(Ice::Float(0.6));
		}
		DamaExecutor::getInstance()->meka->setStiffness("right_arm", stiffnessArm);
		DamaExecutor::getInstance()->meka->setSlewRateProportion("right_arm", slewRateArm);

		// Set Stiffness and SlewRate of the Torso
		robot::SequenceReal stiffnessTorso = robot::SequenceReal();
		robot::SequenceReal slewRateTorso = robot::SequenceReal();
		for(int i=0; i<3; ++i)
		{
			stiffnessTorso.push_back(Ice::Float(0.7));	// 0.6
			slewRateTorso.push_back(Ice::Float(0.5));	// 0.4
		}
		DamaExecutor::getInstance()->meka->setStiffness("torso", stiffnessTorso);
		DamaExecutor::getInstance()->meka->setSlewRateProportion("torso", slewRateTorso);

		DamaExecutor::getInstance()->meka->setHandPayload(Ice::Float(DamaExecutor::getInstance()->payloadHandNormal));

	} catch (const Ice::Exception& ex) {
		std::cerr << ex << std::endl;
		return false;
	} catch (const char* msg) {
		std::cerr << msg << std::endl;
		return false;
	}

	return true;
}

bool DamaExecutor::followPathRobot() const
{
	::std::string pickupPrefix = "Pickup ";
	::std::string transferPrefix = "Transfer-Rigid ";
	::std::string pushPrefix = "Push";

	rl::plan::VectorList::iterator v = DamaExecutor::getInstance()->damaPathFinal.begin();
	::std::vector< ::std::string >::iterator a = DamaExecutor::getInstance()->damaPathFinalAction.begin();
	::std::string lastAction = "NULL";
	for(; v != DamaExecutor::getInstance()->damaPathFinal.end() && a != DamaExecutor::getInstance()->damaPathFinalAction.end(); ++v, ++a)
	{
		// TODO: set stiffness and slewRate according to action type?
		//if((*a) == "Transit") ...

		if((a->substr(0, transferPrefix.size()) != transferPrefix) && (lastAction.substr(0, transferPrefix.size()) == transferPrefix))
		{
			cout << "do hand release ... ";
			cout.flush();

			// decrease payload incrementally
			float payloadHandCurr = DamaExecutor::getInstance()->payloadHandGrasp;
			while(payloadHandCurr - 0.1f >= DamaExecutor::getInstance()->payloadHandNormal - 1.0e-3f)
			{
				payloadHandCurr -= 0.1f;
				DamaExecutor::getInstance()->meka->setHandPayload(Ice::Float(payloadHandCurr));
				usleep(0.1 * 1000 * 1000);
			}

			DamaExecutor::getInstance()->meka->doHandRelease();
			usleep(1.0 * 1000 * 1000);
			DamaExecutor::getInstance()->meka->doHandPushPos();
			usleep(2.7 * 1000 * 1000);
			cout << "done." << endl;
		}

		if((a->substr(0, pickupPrefix.size()) == pickupPrefix) && (lastAction.substr(0, pickupPrefix.size()) != pickupPrefix))
		{
			/*cout << "hand grasp: " << *a << endl;
			cout << *v << endl;*/

			cout << "do hand grasp ... ";
			cout.flush();
			DamaExecutor::getInstance()->meka->doHandRelease();
			usleep(2.0 * 1000 * 1000);
			DamaExecutor::getInstance()->meka->doHandGrasp();
			usleep(2.5 * 1000 * 1000);

			// increase payload incrementally
			float payloadHandCurr = DamaExecutor::getInstance()->payloadHandNormal;
			while(payloadHandCurr + 0.1f <= DamaExecutor::getInstance()->payloadHandGrasp + 1.0e-3f)
			{
				payloadHandCurr += 0.1f;
				DamaExecutor::getInstance()->meka->setHandPayload(Ice::Float(payloadHandCurr));
				usleep(0.1 * 1000 * 1000);
			}

			cout << "done." << endl;
		}

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

		if(v == DamaExecutor::getInstance()->damaPathFinal.begin())
		{
			// sleep 10 seconds long due to driving to starting position
			cout << "driving to starting position 10 seconds long ... ";
			cout.flush();
			DamaExecutor::getInstance()->meka->doHandPushPos();
			usleep(10 * 1000 * 1000);
			cout << "done." << endl;
		}
		else if((a+1) != DamaExecutor::getInstance()->damaPathFinalAction.end() && (*a) != (*(a+1)))
		{
			cout << "mode change from " << (*a) << " to " << *(a+1) << ", wait 0 seconds long ... ";
			cout.flush();
			//usleep(5 * 1000 * 1000);
			cout << "done." << endl;

			if((a+1)->substr(0, pushPrefix.size()) == pushPrefix)
			{
				cout << "push pause, wait 1.0 seconds long ... ";
				cout.flush();
				usleep(1.0 * 1000 * 1000);
				cout << "done." << endl;
			}
		}
		else
		{
			// sleep sampleTime long
			usleep(DamaExecutor::getInstance()->sampleTime * 1000 * 1000);
		}

		lastAction = *a;
	}

	return true;
}
