// Copyright 2016 Carrie Rebhuhn
#ifndef PLANNING_GRIDGRAPH_H_
#define PLANNING_GRIDGRAPH_H_


// from boost
#include <boost/graph/astar_search.hpp>
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



class mt {  // "my types"
 public:
    // Distance traveled in the GridGraph
    typedef std::pair<int, int> edge;
    typedef std::vector<std::vector<bool> > barrier_grid;

    typedef double distance;

    static const int GRID_RANK = 2;
    typedef boost::grid_graph<GRID_RANK> grid;
    typedef boost::graph_traits<grid>::vertex_descriptor vertex_descriptor;
    typedef boost::graph_traits<grid>::vertices_size_type vertices_size_type;

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
    typedef std::vector<vertex_descriptor> vertex_vector;
    typedef boost::vertex_subset_complement_filter<grid, vertex_set>::type
        filtered_grid;
};

// Euclidean heuristic
class euclidean_heuristic :
    public boost::astar_heuristic<mt::filtered_grid, double> {
 public:
    euclidean_heuristic() {}
    double operator()(mt::vertex_descriptor v) {
        double dx = static_cast<double>(m_goal[0]-v[0]);
        double dy = static_cast<double>(m_goal[1]-v[1]);
        return sqrt(dx*dx + dy*dy);
    }
    mt::vertex_descriptor m_goal;
};

// Manhattan heuristic
class manhattan_heuristic :
    public boost::astar_heuristic<mt::filtered_grid, double> {
 public:
    manhattan_heuristic() {}
    double operator()(mt::vertex_descriptor v) {
        double dx = static_cast<double>(m_goal[0] - v[0]);
        double dy = static_cast<double>(m_goal[1] - v[1]);
        return fabs(dx) + fabs(dy);
    }
    mt::vertex_descriptor m_goal;
};


class GridGraph {
 public:
    ~GridGraph() {}
    // Exception thrown when the goal vertex is found
    struct found_goal {};

    // Visitor that terminates when we find the goal vertex
    struct astar_goal_visitor :public boost::default_astar_visitor {
        // astar_goal_visitor(mt::vertex_descriptor goal):m_goal(goal) {};
        astar_goal_visitor() {}

        void examine_vertex(mt::vertex_descriptor u, const mt::filtered_grid&) {
            if (u == m_goal)
                throw found_goal();
        }

        mt::vertex_descriptor m_goal;
    };

    // CREATION FUNCTIONS
    /*GridGraph() :
        m_grid(create_grid(0, 0)), m_barrier_grid(create_barrier_grid()) {}
    GridGraph(std::size_t x, std::size_t y) :
        m_grid(create_grid(x, y)), m_barrier_grid(create_barrier_grid()) {}*/

    explicit GridGraph(mt::barrier_grid obstacle_map) :
        m_grid(create_grid(obstacle_map.size(), obstacle_map[0].size())),
        m_barrier_grid(create_barrier_grid()),
        m_solution_length(0) {
        int v_index = 0;
        for (size_t y = 0; y < obstacle_map[0].size(); y++) {
            for (size_t x = 0; x < obstacle_map.size(); x++) {
                if (obstacle_map[x][y]) {  // there is an obstacle!
                    mt::vertex_descriptor u = vertex(v_index, m_grid);
                    m_barriers.insert(u);  // insert a barrier!
                }
                v_index++;  // increment v_index even if no barrier added!
            }
        }
    }

    GridGraph(const matrix2d &members, int m1, int m2) :
        m_grid(create_grid(members.size(), members[0].size())),
        m_barrier_grid(create_barrier_grid()) {
        mt::barrier_grid obstacle_map(members.size());
        for (size_t i = 0; i < members.size(); i++) {
            for (size_t j = 0; j < members[i].size(); j++) {
                obstacle_map[i][j] = members[i][j] < 0;
            }
        }

        // This map only shows grid cells of membership m1 and m2 as passable.
        // Others are barriers.
        // Backflow (travel from m2 to m1) is allowed but will tend to be
        // suboptimal, so is improbable.
        int v_index = 0;
        for (size_t y = 0; y < obstacle_map[0].size(); y++) {
            for (size_t x = 0; x < obstacle_map.size(); x++) {
                if (obstacle_map[x][y] ||
                    // there is an obstacle, or wrong membership
                    (members[x][y] != m1 && members[x][y] != m2)) {
                    mt::vertex_descriptor u = vertex(v_index, m_grid);
                    m_barriers.insert(u);  // insert a barrier!
                }
                v_index++;
            }
        }
    }

    // The length of the AStarGrid along the specified dimension.
    mt::vertices_size_type length(std::size_t d) const {
        return m_grid.length(d);
    }

    bool has_barrier(mt::vertex_descriptor u) const {
        return m_barriers.find(u) != m_barriers.end();
    }

    double solve(size_t xsource, size_t ysource, size_t xgoal, size_t ygoal) {
        boost::static_property_map<mt::distance> weight(1);
        // The predecessor map is a vertex-to-vertex mapping.
        typedef boost::unordered_map<mt::vertex_descriptor,
            mt::vertex_descriptor,
            mt::vertex_hash> pred_map;
        pred_map predecessor;
        boost::associative_property_map<pred_map> pred_pmap(predecessor);

        // The distance map is a vertex-to-distance mapping.
        typedef boost::unordered_map<mt::vertex_descriptor,
            mt::distance, mt::vertex_hash> dist_map;
        dist_map distance;
        boost::associative_property_map<dist_map> dist_pmap(distance);

        mt::vertex_descriptor s = { {xsource, ysource} };
        mt::vertex_descriptor g = { {xgoal, ygoal} };
        heuristic.m_goal = g;
        visitor.m_goal = g;
        m_solution.clear();

        try {
            astar_search(m_barrier_grid, s, heuristic,
                boost::weight_map(weight).
                predecessor_map(pred_pmap).
                distance_map(dist_pmap).
                visitor(visitor));
        }
        catch (found_goal fg) {
            (void)fg;
            // Walk backwards from the goal through the predecessor chain adding
            // vertices to the solution path.
            for (mt::vertex_descriptor u = g; u != s; u = predecessor[u])
                m_solution.push_back(u);
            m_solution.push_back(s);
            m_solution_length = distance[g];
            return m_solution_length;
        }
        double maxdist = DBL_MAX;
        return maxdist;
    }

    std::vector<easymath::XY> astar(easymath::XY source, easymath::XY goal) {
        size_t xs = static_cast<size_t>(source.x);
        size_t ys = static_cast<size_t>(source.y);
        size_t xg = static_cast<size_t>(goal.x);
        size_t yg = static_cast<size_t>(goal.x);
        solve(xs, ys, xg, yg);

        std::vector<easymath::XY> soln;
        // for (mt::vertex_vector::iterator it
        // = m_solution.begin(); it != m_solution.end(); ++it) {
        for (boost::array<size_t, 2U> it : m_solution) {
            // soln.push_back(easymath::XY(it->front(), it->back()));
            soln.push_back(easymath::XY(it.front(), it.back()));
        }
        return soln;
    }

    bool solved() const { return !m_solution.empty(); }
    bool solution_contains(mt::vertex_descriptor u) const {
        return std::find(m_solution.begin(), m_solution.end(), u)
            != m_solution.end();
    }

    euclidean_heuristic heuristic;
    astar_goal_visitor visitor;


    // The length of the solution path
    mt::distance m_solution_length;

    void printMap(std::string dir, int label1, int label2) {
        std::stringstream ss;
        ss << dir << label1 << "-" << label2 << ".csv";
        std::ofstream output(ss.str());


        for (size_t i = 0; i < length(0); i++) {
            for (size_t j = 0; j < length(1); j++) {
                // Put the character representing this point
                // in the AStarGrid grid.
                mt::vertex_descriptor u = { {i, j} };
                if (solution_contains(u))
                    output << 2 << ",";
                else if (has_barrier(u))
                    output << 1 << ",";
                else
                    output << 0 << ",";
            }
            output << std::endl;
        }
    }
    // The vertices on a solution path through the AStarGrid
    mt::vertex_vector m_solution;

 private:
    // Create the underlying rank-2 grid with the specified dimensions.
    mt::grid create_grid(std::size_t x, std::size_t y) {
        boost::array<std::size_t, mt::GRID_RANK> lengths = { {x, y} };
        return mt::grid(lengths);
    }
    // Filter the barrier vertices out of the underlying grid.
    mt::filtered_grid create_barrier_grid() {
        return boost::make_vertex_subset_complement_filter(m_grid, m_barriers);
    }

    //! The grid underlying the AStarGrid
    mt::grid m_grid;
    //! The underlying AStarGrid grid with barrier vertices filtered out
    mt::filtered_grid m_barrier_grid;
    //! The barriers in the AStarGrid
    mt::vertex_set m_barriers;
};
#endif  // PLANNING_GRIDGRAPH_H_
