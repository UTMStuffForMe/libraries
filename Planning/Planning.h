// Copyright 2016 Carrie Rebhuhn
#ifndef PLANNING_PLANNING_H_
#define PLANNING_PLANNING_H_

#include <boost/unordered_map.hpp>
#include <boost/graph/astar_search.hpp>
#include <list>
#include <vector>
#include <utility>

namespace Planning {
namespace detail {

//! Exception for graph search termination.
struct found_goal {};

//! Defines the point when a goal is found
template <class GType>
class astar_goal_visitor : public boost::default_astar_visitor {
 public:
    typedef typename boost::graph_traits<GType>::vertex_descriptor VType;
    explicit astar_goal_visitor(VType goal) : m_goal(goal) {}
    astar_goal_visitor() {}
    void examine_vertex(VType u, GType) {
        if (u == m_goal)
            throw found_goal();
    }
    VType m_goal;
};

//! Euclidean heuristic. Requires a class object that has functions that
//! translate vertex descriptor to x and y locations.
template<class G, class Gbase>
class euclidean_heuristic : public boost::astar_heuristic<Gbase, double> {
 public:
    typedef typename Gbase::vertex_descriptor V;
    euclidean_heuristic(G funcs, Gbase g, V goal) :
        m_goal(goal), g(g), funcs(funcs) {}

    double operator()(V v) {
        double dx = funcs.get_x(m_goal) - funcs.get_x(v);
        double dy = funcs.get_y(m_goal) - funcs.get_y(v);
        return sqrt(dx*dx + dy*dy);
    }
    V m_goal;
    Gbase g;
    G funcs;    // Allows access to get_x and get_y functions
};

//! Backend for retrieving the euclidean heuristic.
//! Takes a boost graph object for Gbase, so that template can be deduced.
template <class G, class Gbase, class V>
auto get_euclidean_heuristic(G funcs, Gbase g, V v) {
    return euclidean_heuristic<G, Gbase>(funcs, g, v);
}

//! Gets the bgl named params necessary for astar search
template<class G, class Gbase, class V, class P>
auto get_params(const Gbase& g, const V& goal, P *predecessors) {
    return boost::weight_map(G::weight())
        .predecessor_map(G::pred_pmap(predecessors))
        .distance_map(G::dist_pmap())
        .visitor(detail::astar_goal_visitor<Gbase>(goal));
}

}  // namespace detail

//! Interface for a class to use the astar planning
template <class G, class vertex_base, class vertex_hash>
class IBoostGraph {
 public:
    typedef typename G::vertex_descriptor vertex_descriptor;
    //! Maps are vertex-to-vertex mapping.
    typedef typename boost::unordered_map<vertex_descriptor, vertex_descriptor,
        vertex_hash> pred_map;
    typedef typename boost::unordered_map<vertex_descriptor, double,
        vertex_hash> dist_map;
    virtual vertex_descriptor get_descriptor(vertex_base) = 0;
    virtual vertex_base get_vertex_base(vertex_descriptor) = 0;
    virtual double get_x(vertex_descriptor) = 0;
    virtual double get_y(vertex_descriptor) = 0;

    static auto pred_pmap(pred_map *predecessor) {
        return boost::associative_property_map<pred_map>(*predecessor);
    }
    static auto dist_pmap() {
        dist_map distance;
        return boost::associative_property_map<dist_map>(distance);
    }
    static auto weight() {
        return boost::static_property_map<double>(1);
    }
};

template <class V, class G>
std::list<V> astar(G g, V start, V goal) {
    auto s = g.get_descriptor(start);
    auto e = g.get_descriptor(goal);
    auto h = detail::get_euclidean_heuristic(g, g.g, e);
    G::pred_map predecessor;  // Needed for retrieving path
    auto p = detail::get_params<G>(g.g, e, &predecessor);

    std::list<V> solution;
    try {
        boost::astar_search(g.g, s, h, p);
    }
    catch (detail::found_goal) {
        for (auto u = e; u != s; u = predecessor[u]) {
            V val = g.get_vertex_base(u);
            solution.push_back(val);
        }
        return solution;
    }

    return solution;
}
}  // namespace Planning
#endif  // PLANNING_PLANNING_H_
