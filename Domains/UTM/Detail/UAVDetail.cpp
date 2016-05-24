// Copyright 2016 Carrie Rebhuhn

#include "UAVDetail.h"

using easymath::XY;
using std::map;
using std::list;
using easystl::clear;
using std::vector;

UAVDetail::UAVDetail(XY start_loc, XY end_loc, UTMModes::UAVType t,
    MultiGraph<LinkGraph> highGraph, map<edge, int>* linkIDs,
    UTMModes* params, MultiGraph<GridGraph> lowGraph) :
    loc(start_loc), end_loc(end_loc),
    UAV(lowGraph()->get_membership(start_loc), lowGraph()->get_membership(end_loc),
        t, highGraph, linkIDs, params), lowGraph(lowGraph) {
}

int UAVDetail::curSectorID() {
    return lowGraph()->get_membership(loc);
}


int UAVDetail::endSectorID() {
    return lowGraph()->get_membership(end_loc);
}

void UAVDetail::planAbstractPath() {
    if (!on_internal_link) links_touched.insert(cur_link_ID);
    sectors_touched.insert(curSectorID());

    list<int> high_path;
    int cur_s = curSectorID();
    int end_s = endSectorID();
    if (params->_search_type_mode == UTMModes::SearchDefinition::ASTAR) {
        high_path = Planning::astar(highGraph(type_ID)->g, cur_s, end_s, &highGraph()->get_locations());
    } else {
        //high_path = highGraph(type_ID)->rags(cur_s, end_s);
    }

    if (high_path_prev != high_path) {
        pathChanged = true;
        high_path_prev = high_path;
    } else {
        pathChanged = false;
    }

    next_link_ID = nextLinkID();
}

void UAVDetail::planDetailPath() {
    // Get the high-level path
    high_path_prev = getBestPath();

    // Get the astar low-level path
    XY next_loc = highGraph()->get_vertex_loc(nextSectorID());

    std::list<XY> low_path = Planning::astar(lowGraph(curLinkID())->g, loc, next_loc);

    // Add to target waypoints
    clear(&target_waypoints);
    for (XY i : low_path)
        target_waypoints.push(i);
    target_waypoints.pop();  // removes current location from target
}


void UAVDetail::moveTowardNextWaypoint() {
    if (!target_waypoints.size())
        return;  // return if no waypoints

    for (int i = 0; i < speed; i++) {
        loc = target_waypoints.front();
        target_waypoints.pop();
    }
}