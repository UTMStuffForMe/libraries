// Copyright 2016 Carrie Rebhuhn
#ifndef PLANNING_PLANNING_H_
#define PLANNING_PLANNING_H_

#include <list>


//! Visitor that terminates when we find the goal
template <class GType, class VType>
class astar_goal_visitor : public boost::default_astar_visitor {
public:
    explicit astar_goal_visitor(VType goal) : m_goal(goal) {}
    astar_goal_visitor() {};
    void examine_vertex(VType u, GType) {
        if (u == m_goal)
            throw found_goal();
    }
    VType m_goal;
};

// Euclidean heuristic, using an xy input for the goal position
template<class GType>
class euclidean_heuristic_XY : public boost::astar_heuristic<GType, double> {
public:
    typedef std::pair<double, double> coordinate;
    euclidean_heuristic_XY(coordinate goal_coords):
        m_goal(goal_coords){};

    double operator()(coordinate v) {
        double dx = m_goal[0] - v[0];
        double dy = m_goal[1] - v[1];
        return sqrt(dx*dx + dy*dy);
    }
    coordinate m_goal;
};

// Euclidena heuristic, requiring mapping of index to physical location
template<class GType>
class euclidean_heuristic_index : public boost::astar_heuristic<GType, double> {
public:
    typedef std::pair<double, double> coordinate;
    typedef std::vector<coordinate> coordinates;
    euclidean_heuristic_index(size_t goal, coordinates* locations) :
        m_goal(goal), m_locations(locations) {};
    
    double operator()(size_t v) {
        double dx = m_locations->at(m_goal)[0] - m_locations->at(v)[0];
        double dy = m_locations->at(m_goal)[1] - m_locations->at(v)[1];
        return sqrt(dx*dx + dy*dy);
    }
    size_t m_goal;
    coordinates* m_locations;
};


class Planning{
private:
    //! Exception for graph search termination.
    struct found_goal {};

public:
    template <class GType, class VType>
    static std::list<VType> astar(GType g, VType start, VType goal, boost::astar_heuristic<GType,double> h) {
        std::vector<VType> p(num_vertices(g));
        vector<double> d(num_vertices(g));
        
        try {
            boost::astar_search
            (g, start, h,
                boost::predecessor_map(&p[0]).distance_map(&d[0]).
                visitor(astar_goal_visitor<GType, VType>(goal)));
        }
        catch (found_goal fg) {  // found a path to the goal
            std::list<int> shortest_path;
            for (VType v = goal;; v = p[v]) {
                shortest_path.push_front(v);
                if (p[v] == v)
                    break;
            }
        }

        // Return an empty list on failure.
        return list<VType>();
    }


    template <class GType, class VType>
    static std::list<int> rags(GType g, VType start, VType goal) {
        return std::list<int>();
    }
};

#endif  // PLANNING_PLANNING_H_