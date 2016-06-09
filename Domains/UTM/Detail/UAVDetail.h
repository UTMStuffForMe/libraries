// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_UAVDETAIL_H_
#define DOMAINS_UTM_UAVDETAIL_H_

// STL includes
#include <queue>
#include <map>

// Library includes
#include "Domains/UTM/UAV.h"
#include "Planning/GridGraph.h"

class UAVDetail : public UAV {
public:
    UAVDetail(easymath::XY start_loc, easymath::XY end_loc, UTMModes::UAVType t,
        LinkGraph* highGraph, UTMModes* params, GridGraph* lowGraph);

    // Comparison accessors
    bool has_plan() const { return !target_waypoints.empty(); }
    bool is_next(easymath::XY l) const { return target_waypoints.front() == l; }
    bool is_goal(easymath::XY l) const { return end_loc == l; }
    bool at_boundary();
    bool has_detail_plan() { return !target_waypoints.empty(); };
    bool on_internal_link(int next_link_ID) { return cur_link_ID < params->n_links && next_link_ID >= params->n_links; }
    double distance_to(easymath::XY l) const { return easymath::euclidean_distance(loc, l); }

    // Regular accessors
    easymath::XY get_location() { return loc; }
    //! Gets the sector ID from the current location
    virtual int UAVDetail::get_cur_sector() const { return lowGraph->get_membership(loc); }

    // Mutators
    virtual void planAbstractPath();
    void planDetailPath();
    void moveTowardNextWaypoint();

private:
    //! Physical UAV location
    easymath::XY loc, end_loc;
    //! Low-level waypoints
    std::list<easymath::XY> target_waypoints;
    //! Low-level graph
    GridGraph* lowGraph;


    virtual void clearHistory();
    
    std::vector<easymath::XY> prev_locs;
    std::vector<size_t> prev_secs;
    std::vector<bool> prev_commits;
    std::vector<size_t> prev_mems;
    std::vector<size_t> prev_cur_links;
    std::vector<size_t> prev_next_links;
    std::vector<std::list<easymath::XY> >prev_targ;

};

#endif  // DOMAINS_UTM_UAVDETAIL_H_