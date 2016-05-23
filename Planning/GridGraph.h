// Copyright 2016 Carrie Rebhuhn
#ifndef PLANNING_GRIDGRAPH_H_
#define PLANNING_GRIDGRAPH_H_


// from boost
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/unordered_map.hpp>

// from stl
#include <float.h>
#include <fstream>
#include <utility>
#include <string>
#include <vector>
#include <functional>

// from libraries
#include "../../libraries/Math/easymath.h"

/**
* This is a specialization of a graph for an 8-connected grid.
*/
class GridGraph {
private:
    typedef std::pair<int, int> edge;
    typedef std::vector<std::vector<bool> > barrier_grid;
    static const int GRID_RANK = 2;
    typedef boost::grid_graph<GRID_RANK> grid;
    typedef boost::graph_traits<grid>::vertex_descriptor vertex_descriptor;

    // A hash function for vertices.
    struct vertex_hash :std::unary_function<vertex_descriptor, std::size_t> {
        std::size_t operator()(vertex_descriptor const& u) const {
            std::size_t seed = 0;
            boost::hash_combine(seed, u[0]);
            boost::hash_combine(seed, u[1]);
            return seed;
        }
    };

    typedef boost::unordered_set<vertex_descriptor, vertex_hash> vertex_set;
    typedef boost::vertex_subset_complement_filter<grid, vertex_set>::type filtered_grid;

    //! Maps are vertex-to-vertex mapping.
    typedef boost::unordered_map<vertex_descriptor, vertex_descriptor, vertex_hash> pred_map;
    typedef boost::unordered_map<vertex_descriptor, double, vertex_hash> dist_map;

    //! Should only be called from other constructor
    GridGraph(barrier_grid obstacle_map);

    //! Defines the membership for each cell in the grid
    matrix2d members;

    //! Create the underlying rank-2 grid with the specified dimensions.
    grid create_grid(std::size_t x, std::size_t y);

    //! Filter the barrier vertices out of the underlying grid.
    filtered_grid create_barrier_grid();

    //! The grid underlying the AStarGrid
    grid m_grid;

    //! The underlying AStarGrid grid with barrier vertices filtered out
    filtered_grid m_barrier_grid;

    //! The barriers in the AStarGrid
    vertex_set m_barriers;

public:
    explicit GridGraph(const matrix2d &members);
    ~GridGraph() {}

    //! Adds barriers if a cell does not match membership m1 or m2
    void occlude_nonmembers(int m1, int m2);

    //! Performs A* search from the source to the goal
    //std::vector<easymath::XY> astar(easymath::XY source, easymath::XY goal);

    // Accessor functions
    const int get_membership(easymath::XY p) { return static_cast<int>(members[static_cast<size_t>(p.x)][static_cast<size_t>(p.y)]); }
};
#endif  // PLANNING_GRIDGRAPH_H_
