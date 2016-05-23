// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_UAV_H_
#define DOMAINS_UTM_UAV_H_

// libraries includes
#include "../../Planning/MultiGraph.h"
#include "../../Planning/LinkGraph.h"
#include "UTMModesAndFiles.h"

class UAV {
    /*
    This class is for moving UAVs in the airspace. They interact with the
    environment through planning. Planning is done through boost.
    */
 public:
    typedef std::pair<int, int> edge;

    UAV(int start_mem, int end_mem, UTMModes::UAVType t,
        MultiGraph<LinkGraph> highGraph, std::map<edge, int>* linkIDs,
        UTMModes* params);

    ~UAV() {};

    // Accessor functions
    const size_t get_type() { return type_ID; }
    void set_cur_link_ID(int link_ID) { cur_link_ID = link_ID; }
    //! Gets the sector ID from the location
    virtual int curSectorID() { return mem; }
    //! Gets the sector ID for the desired end location
    virtual int endSectorID() { return mem_end; }

    //! Gets the sector ID from the location
    int nextSectorID(int n = 1);

    //! Gets the link ID from the location and desired next loction.
    //! Throws out of range if internal link.
    int curLinkID();

    //! Gets the link ID of the next 'hop'.
    //! Returns the current link if terminal
    int nextLinkID();

    int getDirection();  // gets the cardinal direction of the UAV

    virtual void planAbstractPath();

    std::list<int> getBestPath();  // does not set anything within the UAV
    bool at_destination() { return mem == mem_end; }
    bool at_link_end() { return t <= 0; }
    void decrement_wait() { t--; }
    int get_wait() { return t; }
    void set_wait(int time) { t = time; }
    void set_cur_sector_ID(int sID) { mem = sID; }
    bool link_touched(int lID) { return links_touched.count(lID) > 0; }
    bool sector_touched(int sID) { return sectors_touched.count(sID) > 0; }

protected:

    UTMModes* params;


    int ID;
    size_t type_ID;
    UTMModes::UAVType type;

    double speed;  // connected to type_ID
    bool pathChanged;
    int mem, mem_end;
    std::list<int> high_path_prev;  // saves the high level path
    std::map<edge, int> *linkIDs;

    int next_link_ID;
    int cur_link_ID;




    // Delay modeling/abstraction mode
    int t;

    // Reward calculation stuff
    std::set<int> sectors_touched;  // the sectors that the UAV has touched...
    std::set<int> links_touched;  // the sectors that the UAV has touched...

    bool currently_in_conflict;
    MultiGraph<LinkGraph> highGraph;  // shared with the simulator (for now);
    bool on_internal_link;

};
#endif  // DOMAINS_UTM_UAV_H_
