// Copyright 2016 Carrie Rebhuhn
#include "LinkGraph.h"

using std::vector;
using easymath::XY;

LinkGraph::LinkGraph(vector<XY> locations_set,
    const vector<edge> &edge_array) :
    locations(locations_set) {
    
    // Create graph
    g = mygraph_t(edge_array.begin(), edge_array.end(), locations.size());
    setWeights(matrix1d(edge_array.size(), 1.0));
}

//! This allows the blocking and unblocking of sectors by making travel
//! through a sector highly suboptimal.
void LinkGraph::blockVertex(int vertexID) {
    // Makes it highly suboptimal to travel to a vertex
    saved_weights = getWeights();

    edge_iter ei, ei_end;
    for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
        if ((*ei).m_target == vertex(vertexID))
            put(boost::edge_weight, g, *ei, 999999.99);
    }
}

void LinkGraph::unblockVertex() {
    setWeights(saved_weights);
}

void LinkGraph::setWeights(matrix1d weights) {
    // iterate over all edge descriptors...
    typedef boost::graph_traits<mygraph_t>::edge_iterator edge_iter;
    edge_iter ei, ei_end;
    int i = 0;

    for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
        put(boost::edge_weight, g, *ei, weights[i++]);
    }
}

matrix1d LinkGraph::getWeights() {
    // iterate over all edge descriptors...
    typedef boost::graph_traits<mygraph_t>::edge_iterator edge_iter;
    edge_iter ei, ei_end;
    matrix1d weights;
    for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
        weights.push_back(get(boost::edge_weight, g, *ei));
    }
    return weights;
}

std::list<int> LinkGraph::astar(int start, int goal) {
    std::vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
    std::vector<cost> d(num_vertices(g));
    try {
        boost::astar_search
        (g, start,
            distance_heuristic<mygraph_t, cost>(locations, goal),
            boost::predecessor_map(&p[0]).distance_map(&d[0]).
            visitor(astar_goal_visitor<vertex>(goal)));
    }
    catch (found_goal fg) {  // found a path to the goal
        (void)fg;
        std::list<int> shortest_path;
        for (vertex v = goal;; v = p[v]) {
            shortest_path.push_front(v);
            if (p[v] == v)
                break;
        }
    }

    // Return an empty list on failure.
    return std::list<int>();
}

void LinkGraph::print_graph_to_file(std::string file_path) {
    std::vector<std::vector<bool> >
        connections_matrix(locations.size(),
            std::vector<bool>(locations.size(), false));

    edge_iter ei, ei_end;
    for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
        connections_matrix[(*ei).m_source][(*ei).m_target] = true;

    std::vector<edge> my_edges;
    for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
        my_edges.push_back(std::make_pair((*ei).m_source, (*ei).m_target));
    }

    std::string CONNECTIONS_FILE = file_path + "connections.csv";
    std::string NODES_FILE = file_path + "nodes.csv";
    std::string EDGES_FILE = file_path + "edges.csv";

    FileOut::print_pair_container(locations, NODES_FILE);
    FileOut::print_pair_container(my_edges, EDGES_FILE);
    FileOut::print_vector(connections_matrix, CONNECTIONS_FILE);
}

const size_t LinkGraph::get_n_vertices() {
    return locations.size();
}

XY LinkGraph::get_vertex_loc(int vertexID) {
    return locations.at(vertexID);
}