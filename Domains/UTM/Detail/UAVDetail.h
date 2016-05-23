// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_UAVDETAIL_H_
#define DOMAINS_UTM_UAVDETAIL_H_

#include "../UAV.h"
#include "../../../Planning/GridGraph.h"

class UAVDetail : public UAV {
public:
    UAVDetail(easymath::XY start_loc, easymath::XY end_loc,
        UTMModes::UAVType t, MultiGraph<LinkGraph> highGraph,
        std::map<edge, int>* linkIDs, UTMModes* params,
        MultiGraph<GridGraph> lowGraph);
    MultiGraph<GridGraph> lowGraph;


    // Physical location of a UAV
    easymath::XY loc;
    easymath::XY end_loc;
    std::queue<easymath::XY> target_waypoints;  // target waypoints, low-level
    void moveTowardNextWaypoint();  // takes a time increment to move over

                                    //! Gets the sector ID from the location
    virtual int curSectorID();
    //! Gets the sector ID for the desired end location
    virtual int endSectorID();

    void planDetailPath();
    virtual void planAbstractPath();
};

#endif  // DOMAINS_UTM_UAVDETAIL_H_
