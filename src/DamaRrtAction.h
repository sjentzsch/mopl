/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#ifndef _DAMA_DAMARRTACTION_H_
#define _DAMA_DAMARRTACTION_H_

#include <boost/graph/adjacency_list.hpp>
#include <CGAL/Search_traits.h>

#include <rl/plan/MatrixPtr.h>
#include <rl/plan/Orthogonal_k_neighbor_search.h>
#include <rl/plan/Planner.h>
#include <rl/plan/Model.h>
#include <rl/plan/Sampler.h>
#include <rl/plan/TransformPtr.h>
#include <rl/plan/VectorPtr.h>

#include "Timer.h"

namespace dama
{
	class Model;
	class Sampler;
	class Verifier;

	/**
	 * Rapidly-Exploring Random Trees.
	 */
	class DamaRrtAction : public ::rl::plan::Planner
	{
	public:
		DamaRrtAction(const ::std::size_t& trees = 1);

		virtual ~DamaRrtAction();

		virtual ::std::string getName() const;

		virtual ::std::size_t getNumEdges() const;

		virtual ::std::size_t getNumVertices() const;

		virtual void getPath(::rl::plan::VectorList& path);

		virtual void reset();

		virtual bool solve();

		/** Configuration step size. */
		::rl::math::Real delta;

		/** Epsilon for configuration comparison. */
		::rl::math::Real epsilon;

		/** Use kd-tree for nearest neighbor search instead of brute-force. */
		bool kd;

		::rl::plan::Sampler* sampler;
		
	protected:
		struct VertexBundle
		{
			::std::size_t index;
			
			::rl::plan::VectorPtr q;
			
			::rl::math::Real radius;
			
			::rl::plan::TransformPtr t;

			// TODO Andre: ::std::vector< ::rl::math::Transform > objectPoses;
		};

		struct EdgeBundle
		{
			::std::string action;
		};

		struct TreeBundle;

		typedef ::boost::adjacency_list<
			::boost::listS,
			::boost::listS,
			::boost::bidirectionalS,
			VertexBundle,
			EdgeBundle,
			TreeBundle
		> Tree;

		typedef ::boost::adjacency_list_traits<
			::boost::listS,
			::boost::listS,
			::boost::bidirectionalS,
			::boost::listS
		>::vertex_descriptor Vertex;

		typedef ::std::pair< const ::rl::math::Vector*, Vertex > QueryItem;

		struct CartesianIterator
		{
			typedef const ::rl::math::Real* result_type;
			
			const ::rl::math::Real* operator()(const QueryItem& p) const;
			
			const ::rl::math::Real* operator()(const QueryItem& p, const int&) const;
		};

		struct Distance
		{
			typedef QueryItem Query_item;
			
			Distance();
			
			Distance(::rl::plan::Model* model);
			
			template< typename SearchTraits > ::rl::math::Real max_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< SearchTraits >& r) const;
			
			template< typename SearchTraits > ::rl::math::Real min_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< SearchTraits >& r) const;
			
			::rl::math::Real min_distance_to_rectangle(const ::rl::math::Real& q, const ::rl::math::Real& min, const ::rl::math::Real& max, const ::std::size_t& cutting_dimension) const;
			
			::rl::math::Real new_distance(const ::rl::math::Real& dist, const ::rl::math::Real& old_off, const ::rl::math::Real& new_off, const int& cutting_dimension) const;
			
			::rl::math::Real transformed_distance(const ::rl::math::Real& d) const;
			
			::rl::math::Real transformed_distance(const Query_item& q1, const Query_item& q2) const;
			
			::rl::plan::Model* model;
		};

		typedef ::CGAL::Search_traits< ::rl::math::Real, QueryItem, const ::rl::math::Real*, CartesianIterator > SearchTraits;

		typedef ::rl::plan::Orthogonal_k_neighbor_search< SearchTraits, Distance > NeighborSearch;

		typedef NeighborSearch::Tree NeighborSearchTree;

		typedef ::std::shared_ptr< NeighborSearchTree > NeighborSearchTreePtr;

		typedef ::std::vector< NeighborSearchTreePtr > NearestNeighbors;

		struct TreeBundle
		{
			NearestNeighbors nn;
		};

		typedef ::boost::graph_traits< Tree >::edge_descriptor Edge;

		typedef ::boost::graph_traits< Tree >::edge_iterator EdgeIterator;

		typedef ::std::pair< EdgeIterator, EdgeIterator > EdgeIteratorPair;

		typedef ::boost::graph_traits< Tree >::vertex_iterator VertexIterator;

		typedef ::std::pair< VertexIterator, VertexIterator > VertexIteratorPair;

		typedef ::std::pair< Vertex, ::rl::math::Real > Neighbor;

		virtual Edge addEdge(const Vertex& u, const Vertex& v, Tree& tree);

		void addPoint(NearestNeighbors& nn, const QueryItem& p);

		Vertex addVertex(Tree& tree, const ::rl::plan::VectorPtr& q);

		bool areEqual(const ::rl::math::Vector& lhs, const ::rl::math::Vector& rhs) const;

		virtual void choose(::rl::math::Vector& chosen);

		virtual Vertex connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

		virtual Vertex extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

		virtual Neighbor nearest(const Tree& tree, const ::rl::math::Vector& chosen);

		::std::vector< Vertex > begin;

		::std::vector< Vertex > end;

		::std::vector< Tree > tree;
		
		::dama::Timer timer;

	private:

	};
}

#endif // _DAMA_DAMARRTACTION_H_
