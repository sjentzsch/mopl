/*
 * Copyright (c) 2015, Sören Jentzsch, Markus Rickert, Andre Gaschler
 * (See accompanying file LICENSE.md)
 */

#include "DamaRrtAction.h"
#include <rl/plan/Sampler.h>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/Verifier.h>
#include <rl/plan/Viewer.h>

#if _MSC_VER < 1600
#define nullptr NULL // TODO
#endif

namespace dama
{
	DamaRrtAction::DamaRrtAction(const ::std::size_t& trees) :
		Planner(),
		delta(1.0f),
		epsilon(1.0e-3f),
		kd(true),
		sampler(NULL),
		begin(trees, NULL),
		end(trees, NULL),
		tree(trees)
	{
	}

	DamaRrtAction::~DamaRrtAction()
	{
	}

	DamaRrtAction::Edge
	DamaRrtAction::addEdge(const Vertex& u, const Vertex& v, Tree& tree)
	{
		Edge e = ::boost::add_edge(u, v, tree).first;

		if (NULL != this->viewer)
		{
			this->viewer->drawConfigurationEdge(*tree[u].q, *tree[v].q);
		}

		return e;
	}

	void
	DamaRrtAction::addPoint(NearestNeighbors& nn, const QueryItem& p)
	{
		NearestNeighbors::iterator i;

		for (i = nn.begin(); i != nn.end(); ++i)
		{
			if (NULL == *i)
			{
				break;
			}
		}
		
		NeighborSearchTreePtr tree(new NeighborSearchTree());

		if (nn.end() == i)
		{
			i = nn.insert(i, tree);
		}
		else
		{
			*i = tree;
		}
		
		for (NearestNeighbors::iterator j = nn.begin(); j != i; ++j)
		{
			if (NULL != *j)
			{
				(*i)->insert((*j)->begin(), (*j)->end());
				j->reset();
			}
		}
		
		tree->insert(p);
	}

	DamaRrtAction::Vertex
	DamaRrtAction::addVertex(Tree& tree, const ::rl::plan::VectorPtr& q)
	{
		Vertex v = ::boost::add_vertex(tree);
		tree[v].index = ::boost::num_vertices(tree) - 1;
		tree[v].q = q;
		tree[v].radius = ::std::numeric_limits< ::rl::math::Real >::max();

		if (this->kd)
		{
			this->addPoint(tree[::boost::graph_bundle].nn, QueryItem(q.get(), v));
		}

		if (NULL != this->viewer)
		{
			this->viewer->drawConfigurationVertex(*tree[v].q);
		}

		return v;
	}

	bool
	DamaRrtAction::areEqual(const ::rl::math::Vector& lhs, const ::rl::math::Vector& rhs) const
	{
		if (this->model->distance(lhs, rhs) > this->epsilon)
		{
			return false;
		}
		else
		{
			return true;
		}
	}

	void
	DamaRrtAction::choose(::rl::math::Vector& chosen)
	{
		this->sampler->generate(chosen);
	}

	DamaRrtAction::Vertex
	DamaRrtAction::connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
	{
		::rl::math::Real distance = nearest.second;
		::rl::math::Real step = distance;
		
		bool reached = false;

		if (step <= this->delta)
		{
			reached = true;
		}
		else
		{
			step = this->delta;
		}
		
		::rl::plan::VectorPtr last = ::std::make_shared< ::rl::math::Vector >(this->model->getDof());

		this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *last);

		if (NULL != this->viewer)
		{
//				this->viewer->drawConfiguration(*last);
		}
		
		this->model->setPosition(*last);
		this->model->updateFrames();

		if (this->model->isColliding())
		{
			return NULL;
		}
		
		::rl::math::Vector next(this->model->getDof());

		while (!reached)
		{
			distance = this->model->distance(*last, chosen);
			step = distance;
			
			if (step <= this->delta)
			{
				reached = true;
			}
			else
			{
				step = this->delta;
			}
			
			this->model->interpolate(*last, chosen, step / distance, next);
			
			if (NULL != this->viewer)
			{
//					this->viewer->drawConfiguration(next);
			}
			
			this->model->setPosition(next);
			this->model->updateFrames();
			
			if (this->model->isColliding())
			{
				break;
			}
			
			*last = next;
		}
		
		Vertex connected = this->addVertex(tree, last);
		this->addEdge(nearest.first, connected, tree);
		return connected;
	}

	DamaRrtAction::Vertex
	DamaRrtAction::extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen)
	{
		::rl::math::Real distance = nearest.second;
		::rl::math::Real step = ::std::min(distance, this->delta);

		::rl::plan::VectorPtr next = ::std::make_shared< ::rl::math::Vector >(this->model->getDof());

		this->model->interpolate(*tree[nearest.first].q, chosen, step / distance, *next);

		this->model->setPosition(*next);
		this->model->updateFrames();

		if (!this->model->isColliding())
		{
			Vertex extended = this->addVertex(tree, next);
			this->addEdge(nearest.first, extended, tree);
			return extended;
		}
		
		return NULL;
	}

	::std::string
	DamaRrtAction::getName() const
	{
		return "RRT";
	}

	::std::size_t
	DamaRrtAction::getNumEdges() const
	{
		::std::size_t edges = 0;

		for (::std::size_t i = 0; i < this->tree.size(); ++i)
		{
			edges += ::boost::num_edges(this->tree[i]);
		}
		
		return edges;
	}

	::std::size_t
	DamaRrtAction::getNumVertices() const
	{
		::std::size_t vertices = 0;

		for (::std::size_t i = 0; i < this->tree.size(); ++i)
		{
			vertices += ::boost::num_vertices(this->tree[i]);
		}
		
		return vertices;
	}

	void
	DamaRrtAction::getPath(::rl::plan::VectorList& path)
	{
		Vertex i = this->end[0];
		
		while (i != this->begin[0])
		{
			path.push_front(*this->tree[0][i].q);
			i = ::boost::source(*::boost::in_edges(i, this->tree[0]).first, this->tree[0]);
		}
		
		path.push_front(*this->tree[0][i].q);
	}

	DamaRrtAction::Neighbor
	DamaRrtAction::nearest(const Tree& tree, const ::rl::math::Vector& chosen)
	{
		Neighbor p(nullptr, ::std::numeric_limits< ::rl::math::Real >::max());

		if (this->kd)
		{
			QueryItem query(&chosen, nullptr);
			
			for (NearestNeighbors::const_iterator i = tree[::boost::graph_bundle].nn.begin(); i != tree[::boost::graph_bundle].nn.end(); ++i)
			{
				if (NULL != *i)
				{
					NeighborSearch search(
						*i->get(),
						query,
						1,
						0,
						true,
						Distance(this->model)
					);
					
					if (search.begin()->second < p.second)
					{
						p.first = search.begin()->first.second;
						p.second = search.begin()->second;
					}
				}
			}
		}
		else
		{
			for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first)
			{
				::rl::math::Real d = this->model->transformedDistance(*tree[*i.first].q, chosen);
				
				if (d < p.second)
				{
					p.first = *i.first;
					p.second = d;
				}
			}
		}
		
		p.second = this->model->inverseOfTransformedDistance(p.second);
		
		return p;
	}

	void
	DamaRrtAction::reset()
	{
		for (::std::size_t i = 0; i < this->tree.size(); ++i)
		{
			this->tree[i].clear();
			this->tree[i][::boost::graph_bundle].nn.clear();
			this->begin[i] = NULL;
			this->end[i] = NULL;
		}

		if(this->viewer != NULL)
			this->viewer->reset();
	}

	bool
	DamaRrtAction::solve()
	{
		this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared< ::rl::math::Vector >(*this->start));
		
		::rl::math::Vector chosen(this->model->getDof());
		
		timer.start();
		timer.stop();
		
		while (timer.elapsedDuration() < this->duration)
		{
			this->choose(chosen);

			Neighbor nearest = this->nearest(this->tree[0], chosen);

			Vertex extended = this->extend(this->tree[0], nearest, chosen);
			
			if (NULL != extended)
			{
				if (this->areEqual(*this->tree[0][extended].q, *this->goal))
				{
					this->end[0] = extended;
					return true;
				}
			}
			
			timer.stop();
		}
		
		return false;
	}

	const ::rl::math::Real*
	DamaRrtAction::CartesianIterator::operator()(const QueryItem& p) const
	{
		return p.first->data(); // TODO
	}

	const ::rl::math::Real*
	DamaRrtAction::CartesianIterator::operator()(const QueryItem& p, const int&) const
	{
		return p.first->data() + p.first->size(); // TODO
	}

	DamaRrtAction::Distance::Distance() :
		model(NULL)
	{
	}

	DamaRrtAction::Distance::Distance(::rl::plan::Model* model) :
		model(model)
	{
	}

	template<>
	::rl::math::Real
#if (CGAL_VERSION_NR > 1030801000)
	DamaRrtAction::Distance::max_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< DamaRrtAction::SearchTraits::FT >& r) const
#else
	DamaRrtAction::Distance::max_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< DamaRrtAction::SearchTraits >& r) const
#endif
	{
		::rl::math::Vector min(r.dimension());
		::rl::math::Vector max(r.dimension());
		
		for (int i = 0; i < r.dimension(); ++i)
		{
			min(i) = r.min_coord(i);
			max(i) = r.max_coord(i);
		}
		
		return this->model->maxDistanceToRectangle(*q.first, min, max);
	}

	template<>
	::rl::math::Real
#if (CGAL_VERSION_NR > 1030801000)
	DamaRrtAction::Distance::min_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< DamaRrtAction::SearchTraits::FT >& r) const
#else
	DamaRrtAction::Distance::min_distance_to_rectangle(const Query_item& q, const ::CGAL::Kd_tree_rectangle< DamaRrtAction::SearchTraits >& r) const
#endif
	{
		::rl::math::Vector min(r.dimension());
		::rl::math::Vector max(r.dimension());
		
		for (int i = 0; i < r.dimension(); ++i)
		{
			min(i) = r.min_coord(i);
			max(i) = r.max_coord(i);
		}
		
		return this->model->minDistanceToRectangle(*q.first, min, max);
	}

	::rl::math::Real
	DamaRrtAction::Distance::min_distance_to_rectangle(const ::rl::math::Real& q, const ::rl::math::Real& min, const ::rl::math::Real& max, const ::std::size_t& cutting_dimension) const
	{
		return this->model->minDistanceToRectangle(q, min, max, cutting_dimension);
	}

	::rl::math::Real
	DamaRrtAction::Distance::new_distance(const ::rl::math::Real& dist, const ::rl::math::Real& old_off, const ::rl::math::Real& new_off, const int& cutting_dimension) const
	{
		return this->model->newDistance(dist, old_off, new_off, cutting_dimension);
	}

	::rl::math::Real
	DamaRrtAction::Distance::transformed_distance(const ::rl::math::Real& d) const
	{
		return this->model->transformedDistance(d);
	}

	::rl::math::Real
	DamaRrtAction::Distance::transformed_distance(const Query_item& q1, const Query_item& q2) const
	{
		return this->model->transformedDistance(*q2.first, *q1.first);
	}
}
