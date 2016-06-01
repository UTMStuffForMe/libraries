// Copyright 2016 Carrie Rebhuhn

#include "UAVDetail.h"

#include <list>

using easymath::XY;
using easystl::clear;
using std::list;


UAVDetail::UAVDetail(XY start_loc, XY end_loc, UTMModes::UAVType t,
    LinkGraph* highGraph, UTMModes* params, GridGraph* lowGraph) :
    loc(start_loc), end_loc(end_loc),
    UAV(lowGraph->get_membership(start_loc), lowGraph->get_membership(end_loc),
        t, highGraph, params), lowGraph(lowGraph) {
}

void UAVDetail::planAbstractPath() {
    cur_sector = get_cur_sector();
    sectors_touched.insert(cur_sector);

    if (params->_search_type_mode == UTMModes::SearchDefinition::ASTAR) {
        high_path = Planning::astar<LinkGraph, size_t>
            (highGraph, cur_sector, end_sector);
    } else {
        // high_path = highGraph->at(type_ID)->rags(cur_s, end_s);
    }
}

void UAVDetail::planDetailPath() {
    // Get the high-level path
    high_path = get_best_path();

    // Get the astar low-level path
    XY next_loc = highGraph->get_vertex_loc(get_next_sector());

    list<XY> low_path = Planning::astar<GridGraph, easymath::XY>
        (lowGraph, loc, next_loc);

    // Add to target waypoints
    clear(&target_waypoints);
    for (XY i : low_path)
        target_waypoints.push(i);
    target_waypoints.pop();  // removes current location from target
}


void UAVDetail::moveTowardNextWaypoint() {
    for (int i = 0; i < speed && !target_waypoints.empty(); i++) {
        loc = target_waypoints.front();
        target_waypoints.pop();
    }
}
