// Copyright 2016 Carrie Rebhuhn
#ifndef PLANNING_LINKGRAPH_H_
#define PLANNING_LINKGRAPH_H_

// Boost includes
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>

// STL includes
#include <vector>
#include <list>
#include <utility>
#include <string>

// library includes
#include "../Math/easymath.h"
#include "../FileIO/FileOut.h"

typedef double cost;

// euclidean distance heuristic
template <class Graph, class CostType>
class distance_heuristic : public boost::astar_heuristic<Graph, CostType> {
 public:
    typedef std::vector<easymath::XY> LocMap;
    typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
    distance_heuristic(LocMap locations, Vertex goal) :
        m_location(locations), m_goal(goal)
    {};
    CostType operator()(Vertex u) {
        return CostType(easymath::euclidean_distance(m_location[u],
            m_location[m_goal]));
    }
 private:
    LocMap m_location;
    Vertex m_goal;
};


struct found_goal {};  // exception for termination

// visitor that terminates when we find the goal
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor {
 public:
    explicit astar_goal_visitor(Vertex goal) : m_goal(goal) {}
    template <class Graph>
    void examine_vertex(Vertex u, Graph& ) {
        if (u == m_goal)
            throw found_goal();
    }
 private:
    Vertex m_goal;
};

class LinkGraph {
 private:
    typedef boost::adjacency_list
        <boost::listS,      // edge container
        boost::vecS,        // vertex container
        boost::directedS,   // edge (u,v) can have a different weight than (v,u)
        boost::no_property,
        boost::property<boost::edge_weight_t, cost> > mygraph_t;
    // Note: m_edges is not populated
    // vertex is an int: corresponds to number in the locations list
    typedef mygraph_t::vertex_descriptor vertex;
    typedef mygraph_t::edge_descriptor edge_descriptor;
    typedef mygraph_t::vertex_iterator vertex_iterator;
    typedef std::pair<int, int> edge;
    typedef boost::graph_traits<mygraph_t>::edge_iterator edge_iter;

    matrix1d saved_weights;  // for blocking and unblocking sectors
    mygraph_t g;
    std::vector<easymath::XY> locations;

 public:
    LinkGraph(std::vector<easymath::XY> locations_set,
        const std::vector<edge> &edge_array);
    ~LinkGraph(void) {}
    const size_t get_n_vertices();
    easymath::XY get_vertex_loc(int vertexID);
    void blockVertex(int vertexID);
    void unblockVertex();
    void setWeights(matrix1d weights);
    matrix1d getWeights();
    std::list<int> astar(int start, int goal);
    void print_graph_to_file(std::string file_path);
};
#endif  // PLANNING_LINKGRAPH_H_
