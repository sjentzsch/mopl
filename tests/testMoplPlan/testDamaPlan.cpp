/*
 * Copyright (c) 2015, Andre Gaschler, Sören Jentzsch
 * (See accompanying file LICENSE.md)
 */

#include <fstream>

#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>

#include <rl/mdl/XmlFactory.h>
#include <rl/sg/solid/Scene.h>
#include <rl/plan/Planner.h>
#include <rl/plan/Sampler.h>
#include <rl/sg/solid/Scene.h>

#include "XmlFactory.h"
#include "DamaRrt.h"
#include "DamaModel.h"
#include "DamaSampler.h"

int main(int argc, char** argv)
{
	::dama::XmlFactory damaFactory;
	boost::shared_ptr< ::dama::DamaModel > model = damaFactory.create("data/tasks/config.xml");

	::dama::Timer timer;
	
	/** Configurations */
	//model->debugMode = true;
	
	::std::size_t numRuns = 1;	// 200

	// calculate numRuns seeds (with value in srand you can change the seed set)
	srand(model->groupSeed);
	// srand(time(NULL));
	::std::vector < ::std::size_t > vecSeeds(numRuns, 0);
	for(::std::size_t u=0; u<numRuns; ++u)
		vecSeeds.at(u) = rand();


	// write benchmark header
	std::ofstream benchmarkHeader;
	benchmarkHeader.open("DamaBenchmark.csv", std::ios::app);
	// #iterations: not counted: if sample is unreachable from tree a or is exact a neighbor of tree a or if connected vertex is unreachable from tree b
	benchmarkHeader << "date,robot,nr,seed,solved,tGlobal,tIK,tSampling,tNNSearch,tPropagate,tConnect,iterations,edges,vertices,totalQueries,freeQueries,solVertices,solLength,solLengthMan";
	for(::std::size_t p=0; p<model->vecDamaPrim.size(); ++p)
		benchmarkHeader << "," << model->vecDamaPrim.at(p)->getName() ;
	benchmarkHeader << ::std::endl;
	benchmarkHeader.close();


	::std::vector <bool> vecSolved(numRuns, false);
	::std::vector < ::rl::math::Real > vecTime(numRuns, 0);
	::std::vector < ::std::size_t > vecIterations(numRuns, 0);
	::std::vector < ::std::size_t > vecEdges(numRuns, 0);
	::std::vector < ::std::size_t > vecVertices(numRuns, 0);
	::std::vector < ::std::size_t > vecSolutionVertices(numRuns, 0);
	::std::vector < ::rl::math::Real > vecSolutionLength(numRuns, 0);
	::std::vector < ::rl::math::Real > vecSolutionLengthManipulation(numRuns, 0);
	::std::vector < ::std::vector < ::std::size_t > > vecSolutionManipulationCount(numRuns, ::std::vector < ::std::size_t >(model->vecDamaPrim.size(), 0));


	rl::plan::VectorList path;
	::std::deque< ::std::string > actions;

	/*std::cout << "start: " << (*model->dRrt->start).transpose() << std::endl;
	std::cout << "goal: " << (*model->dRrt->goal).transpose() << std::endl;
	std::cout << "goalDefined: ";
	for(::std::size_t i=0; i<model->dRrt->goalDimDefined->size(); i++)
		std::cout << model->dRrt->goalDimDefined->at(i) << "\t";
	std::cout << std::endl;*/

	for(::std::size_t u=0; u<numRuns; ++u)
	{
		path.clear();
		actions.clear();

		srand(0);
		::std::size_t randSeedNumber = vecSeeds.at(u); //489012115 OR vecSeeds.at(u)
		::boost::mt19937::result_type randSeed = static_cast< ::boost::mt19937::result_type >((double)randSeedNumber);

		if(model->workspaceSampling)
			model->dRrt->setConfig(false, false, false);	// do NOT change this

		model->dRrt->seed(randSeed);
		model->dRrt->resetStatistics();
		model->dRrt->duration = ::std::chrono::duration_cast< ::std::chrono::steady_clock::duration >( ::std::chrono::duration< double >(180.0)); //900.0 //300.0; //1200.0 //boost::lexical_cast< rl::math::Real >(::std::numeric_limits< double >::max());
		// TODO: hierarchical version currently needs this to be redefined ...
		rl::math::Vector startCopy(*(model->dRrt->start));
		model->dRrt->start = &startCopy;
		rl::math::Vector goalCopy(*(model->dRrt->goal));
		model->dRrt->goal = &goalCopy;
		::std::vector < bool > goalDimDefinedCopy(*(model->dRrt->goalDimDefined));
		model->dRrt->goalDimDefined = &goalDimDefinedCopy;
		std::cout << "Start solving nr. " << (u+1) << "/" << numRuns << " with seed " << randSeedNumber << " ... " << std::endl;
		timer.start();
		vecSolved.at(u) = model->dRrt->solveAll(path, actions);
		timer.stop();
		if(vecSolved.at(u))
		{
			::std::string lastAction = "";
			rl::plan::VectorList::iterator i = path.begin();
			rl::plan::VectorList::iterator j = ++path.begin();
			::std::deque< ::std::string >::iterator e = actions.begin();
			for (; i != path.end() && j != path.end() && e != actions.end(); ++i, ++j, ++e)
			{
				::rl::math::Real tempRobotDist = model->cartesianRobotDistance(*i, *j);
				vecSolutionLength.at(u) += tempRobotDist;
				if(*e != ::dama::DamaPrimTransit::getInstance()->getName())
					vecSolutionLengthManipulation.at(u) += tempRobotDist;
				if(lastAction != *e)
				{
					for(::std::size_t p=0; p<vecSolutionManipulationCount.at(u).size(); ++p)
						if(e->substr(0, model->vecDamaPrim.at(p)->getName().size()) == model->vecDamaPrim.at(p)->getName())
							vecSolutionManipulationCount.at(u).at(p) ++;
				}
				lastAction = *e;
			}
		}
		vecTime.at(u) = timer.elapsed();
		vecIterations.at(u) = model->dRrt->getIterationCount();
		vecEdges.at(u) = model->dRrt->getEdgeCount();
		vecVertices.at(u) = model->dRrt->getVertexCount();
		vecSolutionVertices.at(u) = path.size();
		std::cout << "solved: " << vecSolved.at(u) << " \t time: " << vecTime.at(u) << " \t iterations: " << vecIterations.at(u) << " \t edges: " << vecEdges.at(u) << " \t vertices: " << vecVertices.at(u) << " \t vertices (solution): " << vecSolutionVertices.at(u) << " \t length: " << vecSolutionLength.at(u) << " \t length (manipulation only): " << vecSolutionLengthManipulation.at(u) << std::endl;

		// Export statistics to benchmark file
		std::ofstream benchmark;
		benchmark.open("DamaBenchmark.csv", std::ios::app);
#if !QT_KRAMS
		// Current date/time based on current system
		time_t now = time(0);
		// Convert now to tm struct for local timezone
		tm* localtm = localtime(&now);
		benchmark << asctime(localtm) << ",";
#else
		benchmark << QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz").toStdString() << ",";
#endif
		benchmark << model->prefixName << ",";
		benchmark << (u+1) << ",";
		benchmark << randSeedNumber << ",";
		benchmark << (vecSolved.at(u) ? "true" : "false") << ",";
		benchmark << vecTime.at(u) << ",";
		benchmark << model->timeIK << ",";
		benchmark << model->dRrt->getTimeSampling() << ",";
		benchmark << model->dRrt->getTimeNNSearch() << ",";
		benchmark << model->dRrt->getTimePropagate() << ",";
		benchmark << model->dRrt->getTimeConnect() << ",";
		benchmark << vecIterations.at(u) << ",";
		benchmark << vecEdges.at(u) << ",";
		benchmark << vecVertices.at(u) << ",";
		benchmark << model->getTotalQueries() << ",";
		benchmark << model->getFreeQueries() << ",";
		benchmark << vecSolutionVertices.at(u) << ",";
		benchmark << vecSolutionLength.at(u) << ",";
		benchmark << vecSolutionLengthManipulation.at(u) << ",";
		for(::std::size_t p=0; p<vecSolutionManipulationCount.at(u).size(); ++p)
			benchmark << vecSolutionManipulationCount.at(u).at(p) << ",";
		benchmark << std::endl;
		benchmark.close();
	}
	
	
	return EXIT_SUCCESS;
}
