// Copyright 2016 Carrie Rebhuhn
#ifndef PLANNING_PLANNING_H_
#define PLANNING_PLANNING_H_

#include <boost/unordered_map.hpp>
#include <boost/graph/astar_search.hpp>
#include <list>
#include <vector>
#include <utility>

#include "IBoostGraph.h"

namespace Planning {
namespace detail {

//! Exception for graph search termination.
struct found_goal {};

//! Defines the point when a goal is found
template <class G>
class astar_goal_visitor : public boost::default_astar_visitor {
public:
    typedef typename boost::graph_traits<G>::vertex_descriptor V;
    explicit astar_goal_visitor(V goal) : m_goal(goal) {}
    astar_goal_visitor() {}
    void examine_vertex(V u, G) {
        if (u == m_goal)
            throw found_goal();
    }
    V m_goal;
};

//! Euclidean heuristic. Requires a class object that has functions that
//! translate vertex descriptor to x and y locations.
template<class G, class Gbase>
struct euclidean_heuristic : public boost::astar_heuristic<Gbase, double> {
    typedef typename boost::graph_traits<Gbase>::vertex_descriptor V;
    euclidean_heuristic(G* funcs, Gbase g, V goal) :
        m_goal(goal), g(g), funcs(funcs) {}

    double operator()(V v) {
        double dx = funcs->get_x(m_goal) - funcs->get_x(v);
        double dy = funcs->get_y(m_goal) - funcs->get_y(v);
        return sqrt(dx*dx + dy*dy);
    }
    V m_goal;
    Gbase g;
    G* funcs;    // Allows access to get_x and get_y functions
};

//! Backend for retrieving the euclidean heuristic.
//! Takes a boost graph object for Gbase, so that template can be deduced.
template <class G, class Gbase, class V>
auto get_euclidean_heuristic(G* funcs, Gbase g, V v) {
    return euclidean_heuristic<G, Gbase>(funcs, g, v);
}

//! Gets the bgl named params necessary for astar search
template<class G, class Gbase, class V>
auto get_params(G* GraphWrapper, const Gbase&, const V& goal) {
    return boost::weight_map(GraphWrapper->weight)
        .predecessor_map(GraphWrapper->pred_pmap)
        .distance_map(GraphWrapper->dist_pmap)
        .visitor(detail::astar_goal_visitor<Gbase>(goal));
}

}  // namespace detail

template <class G, class V>
std::list<V> astar(G* g, V start, V goal) {
    auto s = g->get_descriptor(start);
    auto e = g->get_descriptor(goal);
    auto h = detail::get_euclidean_heuristic(g, g->g, e);
    auto p = detail::get_params<G>(g,g->g, e);

    std::list<V> solution;
    try {
        boost::astar_search(g->g, s, h, p);
    }
    catch (detail::found_goal) {
        for (auto u = e; u != s; u = g->predecessor[u]) {
            V val = g->get_vertex_base(u);
            solution.push_back(val);
        }
        return solution;
    }

    return solution;
}
}  // namespace Planning
#endif  // PLANNING_PLANNING_H_
