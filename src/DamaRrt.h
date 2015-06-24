/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#ifndef _DAMA_DAMARRT_H_
#define _DAMA_DAMARRT_H_

#include "DamaRrtCon.h"

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <vector>
#include <deque>

namespace dama
{
	class DamaModel;

	class DamaSampler;

	class DamaRrt : public DamaRrtCon
	{
	public:
		DamaRrt(::std::size_t numMovableComponents, ::std::size_t numSupportSurfaces);

		virtual ~DamaRrt();

		virtual ::std::string getName() const;

		virtual void seed(const ::boost::mt19937::result_type& value);

		virtual bool solveAll(::rl::plan::VectorList& path, ::std::deque< ::std::string >& actions);

		virtual bool solve();

		virtual bool connectOld(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen, ::std::string edgeAction, Vertex& vertex);

		virtual bool connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen, ::std::string edgeAction, Vertex& vertex);

		virtual bool isCollisionFree(const ::rl::math::Vector& v1, const ::rl::math::Vector& v2, ::std::string edgeAction, ::rl::plan::VectorList& pathSegment, ::std::deque< ::std::string >& pathSegmentAction);

		virtual Edge addEdge(const Vertex& u, const Vertex& v, ::std::string edgeAction, Tree& tree, bool drawIfPossible = true);

		virtual Vertex addVertex(Tree& tree, const ::rl::plan::VectorPtr& q);

		virtual void getPath(::rl::plan::VectorList& path, ::std::deque< ::std::string >& actions);

		virtual void getObjectPath(::rl::plan::VectorList& path);

		virtual void writeGraphML(Tree& tree, ::std::string fileName);

		virtual void finalizeWork(Vertex* end0, Vertex* end1);

		virtual bool postProc(::rl::plan::VectorList& path, ::std::deque< ::std::string >& actions);

		virtual bool extendPushes(::rl::plan::VectorList& path, ::std::deque< ::std::string >& actions);

		virtual bool pathSmoothing(::rl::plan::VectorList& path, ::std::deque< ::std::string >& actions);

		virtual void completePartialGoal(const ::rl::math::Vector& samplePart, ::rl::math::Vector& goalFull);

		virtual void setConfig(const bool twoTreeVersion, const bool hierarchicalVersion, const bool useAccurateDistance);

		virtual bool getTwoTreeVersion() const {return this->twoTreeVersion;};

		virtual bool getUseAccurateDistance() const {return this->useAccurateDistance;};

		virtual void resetStatistics();

		virtual ::std::size_t getIterationCount() const {return this->iterationCount;};

		virtual ::std::size_t getEdgeCount() const {return this->edgeCount;};

		virtual ::std::size_t getVertexCount() const {return this->vertexCount;};

		virtual ::rl::math::Real getTimeSampling() const {return this->timeSampling;};

		virtual ::rl::math::Real getTimeNNSearch() const {return this->timeNNSearch;};

		virtual ::rl::math::Real getTimePropagate() const {return this->timePropagate;};

		virtual ::rl::math::Real getTimeConnect() const {return this->timeConnect;};

		DamaModel* dModel;

		DamaSampler* dSampler;

		/** step size after which a new vertex/edge will be added during 'connect' */
		::rl::math::Real extendStep;

		/** for each goal dimension defines if dimension should be reached or not */
		::std::vector<bool>* goalDimDefined;

	protected:
		virtual void choose(::rl::math::Vector& chosen, std::vector<bool>& currIsChosen, bool forwardSearch);

		virtual void sampleFromGoalFull(::rl::math::Vector &sample);

		::boost::variate_generator< ::boost::mt19937, ::boost::uniform_int<> > randSubspace;

		::boost::variate_generator< ::boost::mt19937, ::boost::uniform_int<> > randSupportSurface;

		/** true for spanning two trees, first one starting at the start, second one starting at the goal */
		bool twoTreeVersion;

		/** true for using hierarchical version; spanning an object path first and then for each step using the standard dama problem solver */
		bool hierarchicalVersion;

		/** true for using the accurate distance between two configurations. i.e. the robot travel distance (false: use max of euclid subspace distance) */
		bool useAccurateDistance;

		/** FOR STATISTICS, for all (sub)trees together, needs to be reset via resetStatistics() */
		::std::size_t iterationCount;
		::std::size_t edgeCount;
		::std::size_t vertexCount;
		::rl::math::Real timeSampling;
		::rl::math::Real timeNNSearch;
		::rl::math::Real timePropagate;
		::rl::math::Real timeConnect;

		/** File Names */
		::std::string metaFileName;
		::std::string tree0FileName;
		::std::string tree1FileName;
		::std::string treeSolFileName;
		::std::string pathSolVerticesFileName;
		::std::string pathSolEdgesFileName;

	private:

	};
}

#endif /* _DAMA_DAMARRT_H_ */
