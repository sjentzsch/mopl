/*
 * Copyright (c) 2015, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#include <fstream>
#include <QApplication>
#include <QDateTime>

#include "DamaPlanner.h"
#include "XmlFactory.h"

bool DamaPlanner::init(::std::string newTaskFileName)
{
	this->taskFileName = newTaskFileName;

	SoDB::init();

	// TODO: currently we have to initialize model2 before model such that the static primitives get the right reference to model, and not to model2. Fix this?
	// TODO: we have to create the model twice, as e.g. the Viewer class executes commands right on the DamaModel (model2), which should not interfere with the model used for planning
	// TODO: do not create the dModelVis if viewer is turned off ... as soon as you got the order fixed, you can read out dModel to know if viewer is turned off ;)
	::dama::XmlFactory damaFactory;
	::std::cout << "Creating DamaModels according to task \"" << this->getTaskFileName() << "\" ..." << ::std::endl;
	this->dModelVis = damaFactory.create(this->getTaskFileName(), true);
	this->dModel = damaFactory.create(this->getTaskFileName(), false);

	this->dModel->reset();
	this->dModelVis->reset();

	this->startInit = boost::shared_ptr< rl::math::Vector >(this->dModel->dRrt->start);
	this->goalInit = boost::shared_ptr< rl::math::Vector >(this->dModel->dRrt->goal);
	this->goalDimDefinedInit = boost::shared_ptr< ::std::vector<bool> >(this->dModel->dRrt->goalDimDefined);

	// TODO: ?

	return true;
}

bool DamaPlanner::run()
{
	::dama::Timer timer;

	// calculate this->dModel->numRuns seeds (with value in srand you can change the seed set)
	srand(this->dModel->groupSeed);
	// srand(time(NULL));
	::std::vector < ::std::size_t > vecSeeds(this->dModel->numRuns, 0);
	if(this->dModel->seedType == "singleOnly")
	{
		for(::std::size_t u=0; u<this->dModel->numRuns; ++u)
			vecSeeds.at(u) = this->dModel->singleSeed;
	}
	else
	{
		for(::std::size_t u=0; u<this->dModel->numRuns; ++u)
			vecSeeds.at(u) = rand();
	}

	// write benchmark header
	std::ofstream benchmarkHeader;
	benchmarkHeader.open("DamaBenchmark.csv", std::ios::app);
	// #iterations: not counted: if sample is unreachable from tree a or is exact a neighbor of tree a or if connected vertex is unreachable from tree b
	benchmarkHeader << "date,task,nr,singleSeed,solved,tGlobal,tIK,tSampling,tNNSearch,tPropagate,tConnect,iterations,edges,vertices,totalQueries,freeQueries,solVertices,solLength,solLengthMan";
	for(::std::size_t p=0; p<this->dModel->vecDamaPrim.size(); ++p)
		benchmarkHeader << "," << this->dModel->vecDamaPrim.at(p)->getName() ;
	benchmarkHeader << ::std::endl;
	benchmarkHeader.close();


	vecSolved.resize(this->dModel->numRuns, false);
	::std::vector < ::rl::math::Real > vecTime(this->dModel->numRuns, 0);
	::std::vector < ::rl::math::Real > vecIK(this->dModel->numRuns, 0);
	::std::vector < ::rl::math::Real > vecTimeSampling(this->dModel->numRuns, 0);
	::std::vector < ::rl::math::Real > vecTimeNNSearch(this->dModel->numRuns, 0);
	::std::vector < ::rl::math::Real > vecTimePropagate(this->dModel->numRuns, 0);
	::std::vector < ::rl::math::Real > vecTimeConnect(this->dModel->numRuns, 0);
	::std::vector < ::std::size_t > vecIterations(this->dModel->numRuns, 0);
	::std::vector < ::std::size_t > vecEdges(this->dModel->numRuns, 0);
	::std::vector < ::std::size_t > vecVertices(this->dModel->numRuns, 0);
	::std::vector < ::std::size_t > vecTotalQueries(this->dModel->numRuns, 0);
	::std::vector < ::std::size_t > vecFreeQueries(this->dModel->numRuns, 0);
	::std::vector < ::std::size_t > vecSolutionVertices(this->dModel->numRuns, 0);
	::std::vector < ::rl::math::Real > vecSolutionLength(this->dModel->numRuns, 0);
	::std::vector < ::rl::math::Real > vecSolutionLengthManipulation(this->dModel->numRuns, 0);
	::std::vector < ::std::vector < ::std::size_t > > vecSolutionManipulationCount(this->dModel->numRuns, ::std::vector < ::std::size_t >(this->dModel->vecDamaPrim.size(), 0));

	rl::plan::VectorList path;
	::std::deque< ::std::string > actions;

	std::ofstream benchmark;
	benchmark.open("DamaBenchmark.csv", std::ios::app);

	for(::std::size_t u=0; u<this->dModel->numRuns; ++u)
	{
		path.clear();
		actions.clear();

		srand(0);
		::std::size_t randSeedNumber = vecSeeds.at(u); //489012115 OR vecSeeds.at(u)
		::boost::mt19937::result_type randSeed = static_cast< ::boost::mt19937::result_type >((double)randSeedNumber);

		if(this->dModel->workspaceSampling)
			this->dModel->dRrt->setConfig(false, false, false);	// do NOT change this
		else
			this->dModel->dRrt->setConfig(this->dModel->bidirectional, this->dModel->hierarchical, this->dModel->accurateDistance);

		this->dModel->dRrt->seed(randSeed);
		this->dModel->dRrt->resetStatistics();
		//this->dModel->dRrt->duration = 180.0; //900.0 //300.0; //1200.0 //boost::lexical_cast< rl::math::Real >(::std::numeric_limits< double >::max());
		// TODO: hierarchical version currently needs this to be redefined, only because of this they exist ...
		rl::math::Vector startCopy(*(this->startInit.get()));
		this->dModel->dRrt->start = &startCopy;
		rl::math::Vector goalCopy(*(this->goalInit.get()));
		this->dModel->dRrt->goal = &goalCopy;
		::std::vector < bool > goalDimDefinedCopy(*(this->goalDimDefinedInit.get()));
		this->dModel->dRrt->goalDimDefined = &goalDimDefinedCopy;

		// remove me !!!!
		//if(u < this->dModel->numRuns - 1)
		//	continue;

		std::cout << "Start solving run " << (u+1) << "/" << this->dModel->numRuns << " of task \"" << this->dModel->prefixName << "\" with singleSeed " << randSeedNumber << " ... " << std::endl;
		timer.start();
		vecSolved.at(u) = this->dModel->dRrt->solveAll(path, actions);
		timer.stop();
		vecPath.push_back(path);
		vecActions.push_back(actions);
		if(vecSolved.at(u))
		{
			::std::string lastAction = "";
			rl::plan::VectorList::iterator i = path.begin();
			rl::plan::VectorList::iterator j = ++path.begin();
			::std::deque< ::std::string >::iterator e = actions.begin();
			for (; i != path.end() && j != path.end() && e != actions.end(); ++i, ++j, ++e)
			{
				::rl::math::Real tempRobotDist = this->dModel->cartesianRobotDistance(*i, *j);
				vecSolutionLength.at(u) += tempRobotDist;
				if(*e != ::dama::DamaPrimTransit::getInstance()->getName())
					vecSolutionLengthManipulation.at(u) += tempRobotDist;
				if(lastAction != *e)
				{
					for(::std::size_t p=0; p<vecSolutionManipulationCount.at(u).size(); ++p)
						if(e->substr(0, this->dModel->vecDamaPrim.at(p)->getName().size()) == this->dModel->vecDamaPrim.at(p)->getName())
							vecSolutionManipulationCount.at(u).at(p) ++;
				}
				lastAction = *e;
			}
		}
		vecTime.at(u) = timer.elapsed();
		vecIK.at(u) = this->dModel->timeIK;
		vecTimeSampling.at(u) = this->dModel->dRrt->getTimeSampling();
		vecTimeNNSearch.at(u) = this->dModel->dRrt->getTimeNNSearch();
		vecTimePropagate.at(u) = this->dModel->dRrt->getTimePropagate();
		vecTimeConnect.at(u) = this->dModel->dRrt->getTimeConnect();
		vecIterations.at(u) = this->dModel->dRrt->getIterationCount();
		vecEdges.at(u) = this->dModel->dRrt->getEdgeCount();
		vecVertices.at(u) = this->dModel->dRrt->getVertexCount();
		vecTotalQueries.at(u) = this->dModel->getTotalQueries();
		vecFreeQueries.at(u) = this->dModel->getFreeQueries();
		vecSolutionVertices.at(u) = path.size();
		std::cout << "solved: " << vecSolved.at(u) << " \t time: " << vecTime.at(u) << " \t iterations: " << vecIterations.at(u) << " \t edges: " << vecEdges.at(u) << " \t vertices: " << vecVertices.at(u) << " \t vertices (solution): " << vecSolutionVertices.at(u) << " \t length: " << vecSolutionLength.at(u) << " \t length (manipulation only): " << vecSolutionLengthManipulation.at(u) << std::endl;

		// Export statistics to benchmark file
		benchmark << QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz").toStdString() << ",";
		benchmark << this->dModel->prefixName << ",";
		benchmark << (u+1) << ",";
		benchmark << randSeedNumber << ",";
		benchmark << (vecSolved.at(u) ? "true" : "false") << ",";
		benchmark << vecTime.at(u) << ",";
		benchmark << vecIK.at(u) << ",";
		benchmark << vecTimeSampling.at(u) << ",";
		benchmark << vecTimeNNSearch.at(u) << ",";
		benchmark << vecTimePropagate.at(u) << ",";
		benchmark << vecTimeConnect.at(u) << ",";
		benchmark << vecIterations.at(u) << ",";
		benchmark << vecEdges.at(u) << ",";
		benchmark << vecVertices.at(u) << ",";
		benchmark << vecTotalQueries.at(u) << ",";
		benchmark << vecFreeQueries.at(u) << ",";
		benchmark << vecSolutionVertices.at(u) << ",";
		benchmark << vecSolutionLength.at(u) << ",";
		benchmark << vecSolutionLengthManipulation.at(u) << ",";
		for(::std::size_t p=0; p<vecSolutionManipulationCount.at(u).size(); ++p)
			benchmark << vecSolutionManipulationCount.at(u).at(p) << ",";
		benchmark << std::endl;
	}

	if(this->dModel->numRuns > 1)
	{
		// Export average data to benchmark file if you run the same problem more than one

		std::size_t countSolved = 0;
		for(auto it : vecSolved)
			if(it == true)
				countSolved++;

		std::cout << "*** Summary: " << countSolved << "/" << this->dModel->numRuns << " solved ***" << ::std::endl;

		std::cout << "avg of all: \t time: " << getAvgValue(vecTime, false) << " \t iterations: " << getAvgValue(vecIterations, false) << " \t edges: " << getAvgValue(vecEdges, false) << " \t vertices: " << getAvgValue(vecVertices, false) << " \t vertices (solution): " << getAvgValue(vecSolutionVertices, false) << " \t length: " << getAvgValue(vecSolutionLength, false) << " \t length (manipulation only): " << getAvgValue(vecSolutionLengthManipulation, false) << std::endl;

		benchmark << ",";
		benchmark << ",";
		benchmark << ",";
		benchmark << "avg (all),";
		benchmark << countSolved << "/" << this->dModel->numRuns << ",";
		benchmark << getAvgValue(vecTime, false) << ",";
		benchmark << getAvgValue(vecIK, false) << ",";
		benchmark << getAvgValue(vecTimeSampling, false) << ",";
		benchmark << getAvgValue(vecTimeNNSearch, false) << ",";
		benchmark << getAvgValue(vecTimePropagate, false) << ",";
		benchmark << getAvgValue(vecTimeConnect, false) << ",";
		benchmark << getAvgValue(vecIterations, false) << ",";
		benchmark << getAvgValue(vecEdges, false) << ",";
		benchmark << getAvgValue(vecVertices, false) << ",";
		benchmark << getAvgValue(vecTotalQueries, false) << ",";
		benchmark << getAvgValue(vecFreeQueries, false) << ",";
		benchmark << getAvgValue(vecSolutionVertices, false) << ",";
		benchmark << getAvgValue(vecSolutionLength, false) << ",";
		benchmark << getAvgValue(vecSolutionLengthManipulation, false) << ",";
		for(::std::size_t p=0; p<vecSolutionManipulationCount.back().size(); ++p)
		{
			::rl::math::Real avgValue = 0.0;
			for(::std::size_t r=0; r<this->dModel->numRuns; ++r)
				avgValue += vecSolutionManipulationCount.at(r).at(p);
			avgValue /= this->dModel->numRuns;
			benchmark << avgValue << ",";
		}
		benchmark << std::endl;

		std::cout << "avg solved: \t time: " << getAvgValue(vecTime, true) << " \t iterations: " << getAvgValue(vecIterations, true) << " \t edges: " << getAvgValue(vecEdges, true) << " \t vertices: " << getAvgValue(vecVertices, true) << " \t vertices (solution): " << getAvgValue(vecSolutionVertices, true) << " \t length: " << getAvgValue(vecSolutionLength, true) << " \t length (manipulation only): " << getAvgValue(vecSolutionLengthManipulation, true) << std::endl;

		benchmark << ",";
		benchmark << ",";
		benchmark << ",";
		benchmark << "avg (solved),";
		benchmark << countSolved << "/" << this->dModel->numRuns << ",";
		benchmark << getAvgValue(vecTime, true) << ",";
		benchmark << getAvgValue(vecIK, true) << ",";
		benchmark << getAvgValue(vecTimeSampling, true) << ",";
		benchmark << getAvgValue(vecTimeNNSearch, true) << ",";
		benchmark << getAvgValue(vecTimePropagate, true) << ",";
		benchmark << getAvgValue(vecTimeConnect, true) << ",";
		benchmark << getAvgValue(vecIterations, true) << ",";
		benchmark << getAvgValue(vecEdges, true) << ",";
		benchmark << getAvgValue(vecVertices, true) << ",";
		benchmark << getAvgValue(vecTotalQueries, true) << ",";
		benchmark << getAvgValue(vecFreeQueries, true) << ",";
		benchmark << getAvgValue(vecSolutionVertices, true) << ",";
		benchmark << getAvgValue(vecSolutionLength, true) << ",";
		benchmark << getAvgValue(vecSolutionLengthManipulation, true) << ",";
		for(::std::size_t p=0; p<vecSolutionManipulationCount.back().size(); ++p)
		{
			::rl::math::Real avgValue = 0.0;
			::std::size_t numHits = 0;
			for(::std::size_t r=0; r<this->dModel->numRuns; ++r)
			{
				if(vecSolved.at(r))
				{
					avgValue += vecSolutionManipulationCount.at(r).at(p);
					numHits++;
				}
			}
			avgValue /= numHits;
			benchmark << avgValue << ",";
		}
		benchmark << std::endl;
	}

	benchmark.close();

	return true;
}

template <typename T> ::rl::math::Real DamaPlanner::getAvgValue(::std::vector< T > vecValue, bool considerOnlySolvedRuns) const
{
	::rl::math::Real avgValue = 0.0;
	::std::size_t numHits = 0;
	for(::std::size_t i=0; i<vecValue.size(); i++)
	{
		if(!considerOnlySolvedRuns || vecSolved.at(i))
		{
			avgValue += vecValue.at(i);
			numHits++;
		}
	}
	avgValue /= numHits;
	return avgValue;
}

::std::string DamaPlanner::getTaskFileName() const
{
	return this->taskFileName;
}
