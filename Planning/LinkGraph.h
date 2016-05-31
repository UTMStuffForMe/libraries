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

typedef boost::adjacency_list
<boost::listS,      // edge container
    boost::vecS,        // vertex container
    boost::directedS,   // edge (u,v) can have a different weight than (v,u)
    boost::no_property,
    boost::property<boost::edge_weight_t, double> > mygraph_t;

typedef boost::hash<mygraph_t::vertex_descriptor> node_hash;

typedef IBoostGraph<mygraph_t, size_t, node_hash> LinkBase;

class LinkGraph : public LinkBase {
 public:
    //! For compliance with base type
    typedef typename LinkBase::vertex_descriptor vertex_descriptor;
    typedef typename LinkBase::dist_map dist_map;
    typedef typename LinkBase::pred_map pred_map;
    vertex_descriptor get_descriptor(size_t v) { return v; }
    size_t get_vertex_base(vertex_descriptor v) { return v; }
    double get_x(vertex_descriptor v) {
            return locations[v].x;
    }
    double get_y(vertex_descriptor v) { return locations[v].y; }

    // More convenient types
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
