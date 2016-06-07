// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_UAV_H_
#define DOMAINS_UTM_UAV_H_

// STL includes
#include <list>
#include <set>
#include <utility>
#include <map>
#include <vector>

// libraries includes
#include "Planning/LinkGraph.h"
#include "UTMModesAndFiles.h"

class UAV {
    /*
    This class is for moving UAVs in the airspace. They interact with the
    environment through planning. Planning is done through boost.
    */

 protected:
    // Typedefs
    typedef std::pair<int, int> edge;


    // Constant
    const UTMModes* params;
    const size_t type_ID;
    const double speed;  // connected to type_ID
    const int end_sector;
    LinkGraph* highGraph;  //! shared with the simulator (for now)--non-constant subfunctions
public:
    int ID;  //! const in run, but based on non-constant variable: made public temporarily
protected:

    // Non-constant
    int cur_sector, next_sector;
    std::list<size_t> high_path;
    int cur_link_ID;
    int t;
    std::set<int> sectors_touched, links_touched;

    //! Protected  Accessor functions    
    //! Gets the sector ID for the desired end location
    static int get_next_sector(const std::list<size_t>& high_path) {
        // This will throw an out of bounds error if high_path is empty
        if (high_path.size() < 2) return high_path.front();
        else return *std::next(high_path.begin());
    }
    std::list<size_t> get_best_path() const{ return Planning::astar<LinkGraph, size_t>(highGraph, cur_sector, end_sector); }


public:
    UAV(int start_sector, int end_sector, UTMModes::UAVType t,
        LinkGraph* highGraph, UTMModes* params);

    ~UAV() {};

    // Accessors
    //! Gets the stored private variable
    virtual int get_cur_sector() const { return cur_sector; }
    size_t get_type() const { return type_ID; }
    //! Gets the link ID from the location and desired next loction.
    //! Throws out of range if internal link.
    int get_cur_link() const { return cur_link_ID; }
    bool at_link_end() const { return t <= 0; }
    bool sector_touched(int sID) const { return sectors_touched.count(sID) > 0; }
    bool at_destination() const { return cur_sector == end_sector; }
    bool at_terminal_link() const { return high_path.size() <= 2; }
    int get_travel_direction() const { return highGraph->get_direction(cur_sector, get_next_sector()); }
    int get_wait() const { return t; }
    int get_next_sector() const { return get_nth_sector(1); }
    int get_nth_sector(int n) const { return *std::next(high_path.begin(), n); } // zero indexed
    bool link_touched(int lID) const { return links_touched.count(lID) > 0; }

    
    //! Mutators
    //! Sets the current link ID based on passed value
    virtual void planAbstractPath();
    void set_cur_link_ID(int link_ID) { cur_link_ID = link_ID; }
    void set_cur_sector_ID(int sID) { cur_sector = sID; }
    void set_wait(int time) { t = time; }
    void decrement_wait() { t--; }

    // ~B
    std::vector<int> prev_cur_links;
    std::vector<int> prev_next_links;
    std::vector<std::list<int> > prev_high_path_prev;
    int next_sector_ID; // This gets updated after UAV moves
    int cur_sector_ID; // for debugging
    
        int times_reached_goal;
    
        virtual void clearHistory() {}
    
        bool reached_next_sector = false, reached_goal_sector = false, internal_link = false;
};
#endif  // DOMAINS_UTM_UAV_H_
