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

public:
    size_t ID;  //! const in run, but based on non-constant variable

protected:
    // Typedefs
    typedef std::pair<size_t, size_t> edge;


    // Constant
    const UTMModes* params;
    const size_t type_ID;
    const double speed;  // connected to type_ID
	// Carrie! I had to make this un-const. Can't give a UAV a new goal if it's end_sector can't be reset.
	// If there's a better way to do this, I'm fine with that. ~B
    size_t end_sector;
    LinkGraph* highGraph;  //! shared with the simulator (for now)--non-constant subfunctions

    // Non-constant
    size_t cur_sector, next_sector;
    std::list<size_t> high_path;
    size_t cur_link_ID;
    int t;
    std::set<size_t> sectors_touched, links_touched;

    //! Protected  Accessor functions    
    //! Gets the sector ID for the desired end location
/*    size_t get_next_sector(const std::list<size_t>& high_path) const {
        // This will throw an out of bounds error if high_path is empty
        if (high_path.size() < 2) return high_path.front();
        else return *std::next(high_path.begin());
    }*/
    std::list<size_t> get_best_path() const { return Planning::astar<LinkGraph, size_t>(highGraph, cur_sector, end_sector); }


public:
    UAV(int start_sector, int end_sector, UTMModes::UAVType t,
        LinkGraph* highGraph, UTMModes* params);

    virtual ~UAV() {};

    // Accessors
    //! Gets the stored private variable
    virtual size_t get_cur_sector() const { return cur_sector; }
    size_t get_type() const { return type_ID; }
    //! Gets the link ID from the location and desired next loction.
    //! Throws out of range if internal link.
    size_t get_cur_link() const { return cur_link_ID; }
    bool at_link_end() const { return t <= 0; }
    bool sector_touched(size_t sID) const { return sectors_touched.count(sID) > 0; }
    bool at_terminal_link() const { return high_path.size() <= 2; }
    int get_travel_direction() const { return highGraph->get_direction(cur_sector, get_next_sector()); }
    int get_wait() const { return t; }
    size_t get_next_sector() const { return get_nth_sector(1); }
	// Carrie! I had to change this slightly for detailed sim. If I remember correctly, it was throwing errors because sometimes iterator was invalid
	//  Maybe it's not really needed. (Sorry, it was a while ago, memory's fuzzy)
    size_t get_nth_sector(int n) const { return high_path.size() > n ? *std::next(high_path.begin(), n) : *std::next(high_path.begin(), high_path.size() - 1); } // zero indexed
    bool link_touched(size_t lID) const { return links_touched.count(lID) > 0; }


    //! Mutators
    //! Sets the current link ID based on passed value
    virtual void planAbstractPath();
    void set_cur_link_ID(size_t link_ID) { cur_link_ID = link_ID; }
    void set_cur_sector_ID(size_t sID) {
        cur_sector = sID;
    }
    void set_wait(int time) {
        if (time < 0) {
            printf("bad");
        }
        t = time;
    }
    void decrement_wait() { t--; }

	// Carrie! I don't need these anymore if that's what you left them here for
    size_t next_sector_ID; // This gets updated after UAV moves
    size_t cur_sector_ID; // for debugging
};
#endif  // DOMAINS_UTM_UAV_H_
