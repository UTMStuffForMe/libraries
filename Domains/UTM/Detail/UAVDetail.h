// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_UAVDETAIL_H_
#define DOMAINS_UTM_UAVDETAIL_H_

// STL includes
#include <queue>
#include <map>

// Library includes
#include "../UAV.h"
#include "../../../Planning/GridGraph.h"

class UAVDetail : public UAV {
public:
    UAVDetail(easymath::XY start_loc, easymath::XY end_loc, UTMModes::UAVType t,
        LinkGraph* highGraph, UTMModes* params, GridGraph* lowGraph);
    GridGraph* lowGraph;


    // Physical location of a UAV
    easymath::XY loc;
    easymath::XY end_loc;
    std::queue<easymath::XY> target_waypoints;  // target waypoints, low-level
    void moveTowardNextWaypoint();

    //! Accessor functions
    //! Gets the sector ID from the current location
    virtual int UAVDetail::get_cur_sector() const { return lowGraph->get_membership(loc); }

    void planDetailPath();
    virtual void planAbstractPath();
};

#endif  // DOMAINS_UTM_UAVDETAIL_H_
