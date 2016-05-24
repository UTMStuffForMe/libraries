// Copyright 2016 Carrie Rebhuhn
#ifndef PLANNING_LINKGRAPH_H_
#define PLANNING_LINKGRAPH_H_

// Boost includes
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>

// STL includes
#include <vector>
#include <utility>
#include <string>
#include <map>

// library includes
#include "../Math/easymath.h"
#include "../FileIO/FileOut.h"
#include "Planning.h"

class LinkGraph {
 private:
    typedef double cost;
    typedef boost::adjacency_list
        <boost::listS,      // edge container
        boost::vecS,        // vertex container
        boost::directedS,   // edge (u,v) can have a different weight than (v,u)
        boost::no_property,
        boost::property<boost::edge_weight_t, cost> > mygraph_t;
    // Note: m_edges is not populated
    // vertex is an int: corresponds to number in the locations list
    typedef mygraph_t::vertex_descriptor vertex_descriptor;
    typedef mygraph_t::edge_descriptor edge_descriptor;
    typedef mygraph_t::vertex_iterator vertex_iterator;
    typedef std::pair<int, int> edge;
    typedef boost::graph_traits<mygraph_t>::edge_iterator edge_iter;

    matrix1d saved_weights;  // for blocking and unblocking sectors
    std::vector<easymath::XY> locations;
    std::map<easymath::XY, int> loc2mem;  // maps location to membership

    void blockVertex(int vertexID);
    void unblockVertex();
    bool fully_connected();  // Tests whether the graph is fully connected
    bool intersects_existing_edge(edge candidate);


 public:
     mygraph_t g;

     LinkGraph(std::vector<easymath::XY> locations_set,
        const std::vector<edge> &edge_array);
    LinkGraph(size_t n_vertices, size_t xdim, size_t ydim);
    ~LinkGraph(void) {}


    //! Accessor functions
    const size_t get_n_vertices() { return locations.size(); }
    const size_t get_n_edges() { return num_edges(g); }
    const easymath::XY get_vertex_loc(int vID) { return locations.at(vID); }
    const int get_membership(easymath::XY pt) { return loc2mem.at(pt); }
    const matrix1d get_weights();
    const std::vector<edge> get_edges();
    const std::vector<easymath::XY> get_locations() { return locations; }
    void set_weights(matrix1d weights);

    const int get_direction(int m1, int m2);

    //! Printout
    void print_graph(std::string file_path);
};
#endif  // PLANNING_LINKGRAPH_H_
