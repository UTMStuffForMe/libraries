// Copyright 2016 Carrie Rebhuhn
#ifndef PLANNING_PLANNING_H_
#define PLANNING_PLANNING_H_

#include <boost/graph/astar_search.hpp>
#include <list>
#include <vector>
#include <utility>


//! Visitor that terminates when we find the goal
namespace Planning {

namespace backend {

//! Exception for graph search termination.
struct found_goal {};

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

// Euclidean heuristic, using an 2d input for the goal position
template<class GType, class VType>
class euclidean_heuristic_v2d : public boost::astar_heuristic<GType, double> {
public:
    explicit euclidean_heuristic_v2d(VType goal_coords) :
        m_goal(goal_coords) {}

    double operator()(VType v) {
        double dx = m_goal[0] - v[0];
        double dy = m_goal[1] - v[1];
        return sqrt(dx*dx + dy*dy);
    }
    VType m_goal;
};

// Euclidean heuristic, requiring mapping of index to physical location
template<class GType, class LocMap>
class euclidean_heuristic_indexed :
    public boost::astar_heuristic<GType, double> {
public:
    typedef typename boost::graph_traits<GType>::vertex_descriptor VType;
    euclidean_heuristic_indexed(VType goal, LocMap* locations) :
        m_goal(goal), m_locations(locations) {}

    double operator()(VType v) {
        double dx = m_locations->at(m_goal).x - m_locations->at(v).x;
        double dy = m_locations->at(m_goal).y - m_locations->at(v).y;
        return sqrt(dx*dx + dy*dy);
    }
    VType m_goal;
    LocMap* m_locations;
};

template <class GType, class VType, class HType>
static std::list<VType> astar(GType g, VType start, VType goal, HType h) {
    std::vector<VType> p(num_vertices(g));
    std::vector<double> d(num_vertices(g));

    try {
        boost::astar_search
            (g, start, h,
                boost::predecessor_map(&p[0]).distance_map(&d[0]).
                visitor(astar_goal_visitor<GType>(goal)));
    }
    catch (found_goal) {  // found a path to the goal
        std::list<VType> shortest_path;
        for (VType v = goal;; v = p[v]) {
            shortest_path.push_front(v);
            if (p[v] == v)
                break;
            return shortest_path;
        }
    }
    return std::list<VType>();
}

}

template <class GType, class VType>
static std::list<VType> astar(GType g, VType start, VType goal) {
    backend::euclidean_heuristic_v2d<GType, VType> h(goal);

    boost::graph_traits<GType>::vertex_descriptor s = { start.x, start.y };
    boost::graph_traits<GType>::vertex_descriptor e = { start.x, start.y };

    auto p = astar(g, s, e, h);
    std::list<VType> out;
    for (auto i : p) {
        out.push_back(VType(i[0], i[1]));
    }
    return out;
}

template <class GType, class VType, class LocMap>
static std::list<VType> astar(GType g, VType start, VType goal, LocMap* locations) {
    std::vector<VType> p(num_vertices(g));
    std::vector<double> d(num_vertices(g));

    backend::euclidean_heuristic_indexed<GType, LocMap> h(goal, locations);
    return backend::astar(g, start, goal, h);
}


/*
template <class GType, class VType>
static std::list<int> rags(GType g, VType start, VType goal) {
    return std::list<int>();
}
*/
}
#endif  // PLANNING_PLANNING_H_
