// Copyright 2016 Carrie Rebhuhn
#include "UAV.h"
#include <map>
#include <list>
#include <vector>
#include "../../STL/easystl.h"

using easystl::clear;
using easymath::XY;
using std::map;
using std::list;
using std::vector;

UAV::UAV(int start_mem, int mem_end, UTMModes::UAVType my_type,
    TypeGraphManager* highGraph, std::map<edge, int>* linkIDs, UTMModes* params) :
    highGraph(highGraph),
    mem(start_mem),
    mem_end(mem_end),
    type_ID(size_t(my_type)),
    type(my_type), speed(1.0),
    linkIDs(linkIDs),
    params(params) {
    static int calls = 0;
    ID = calls++;

    // Get initial plan and update
    planAbstractPath();
    

    // Set the link ID now that the path is known
    set_cur_link_ID(curLinkID());

    // printf("ID = %i, l next = %i\n",ID,next_link_ID);
}


UAVDetail::UAVDetail(XY start_loc, XY end_loc, UTMModes::UAVType t,
    TypeGraphManager* highGraph, map<edge, int>* linkIDs,
    UTMModes* params, SectorGraphManager* lowGraph) :
    loc(start_loc), end_loc(end_loc),
    UAV(lowGraph->getMembership(start_loc), lowGraph->getMembership(end_loc),
        t, highGraph, linkIDs, params), lowGraph(lowGraph) {
}


int UAV::nextSectorID(int n) {
    // Returns the nth sector ID from the current sector, or as far as it can go
    if (!high_path_prev.size())
        return curSectorID();

    int increment = 0;
    if (static_cast<int>(high_path_prev.size()) > n)
        increment = n;
    else
        increment = (high_path_prev.size() - 1);
    return *std::next(high_path_prev.begin(), increment);
}

int UAV::curLinkID() {
    edge link(curSectorID(), nextSectorID());
    if (link.first == link.second) {
        on_internal_link = true;
        return linkIDs->at(link);
    } else {
        on_internal_link = false;
        return linkIDs->at(link);
    }
}

int UAV::curSectorID() {
    return mem;
}
int UAVDetail::curSectorID() {
    return lowGraph->getMembership(loc);
}

int UAV::endSectorID() {
    return mem_end;
}

int UAVDetail::endSectorID() {
    return lowGraph->getMembership(end_loc);
}

int UAV::nextLinkID() {
    if (nextSectorID(1) == nextSectorID(2)) {
        return curLinkID();
    } else {
        edge link(nextSectorID(1), nextSectorID(2));
        return linkIDs->at(link);
    }
}

std::list<int> UAV::getBestPath() {
    return highGraph->astar(mem, mem_end, type_ID);
}

void UAV::planAbstractPath() {
    if (!on_internal_link) links_touched.insert(cur_link_ID);
    sectors_touched.insert(curSectorID());

    list<int> high_path;
    int cur_s = curSectorID();
    int end_s = endSectorID();
    if (params->_search_type_mode == UTMModes::SearchDefinition::ASTAR) {
        high_path = highGraph->astar(cur_s, end_s, type_ID);
    } else {
        high_path = highGraph->rags(cur_s, end_s, type_ID);
    }

    if (high_path.size() == 1) {
        printf("Path not found!");
        system("pause");
    }

    if (high_path_prev != high_path) {
        pathChanged = true;
        high_path_prev = high_path;
        // clear(target_waypoints);
        // for (int sector_id:high_path)
        // target_waypoints.push(highGraph->getLocation(sector_id));




        /*for (int i : high_path) {
            cout << i << ",";
        }
        cout << endl;*/
    } else {
        pathChanged = false;
    }

    next_link_ID = nextLinkID();
}


void UAVDetail::planAbstractPath() {
    if (!on_internal_link) links_touched.insert(cur_link_ID);
    sectors_touched.insert(curSectorID());

    list<int> high_path;
    int cur_s = curSectorID();
    int end_s = endSectorID();
    if (params->_search_type_mode == UTMModes::SearchDefinition::ASTAR) {
        high_path = highGraph->astar(cur_s, end_s, type_ID);
    } else {
        high_path = highGraph->rags(cur_s, end_s, type_ID);
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
    XY next_loc = highGraph->getLocation(nextSectorID());
    vector<XY> low_path = lowGraph->astar(loc, next_loc);
    std::reverse(low_path.begin(), low_path.end());

    // Add to target waypoints
    clear(&target_waypoints);
    for (XY i : low_path)
        target_waypoints.push(i);
    target_waypoints.pop();  // removes current location from target
}

int UAV::getDirection() {
    // Identifies whether traveling in one of four cardinal directions
    XY a = highGraph->getLocation(mem);
    XY b = highGraph->getLocation(nextSectorID());
    return cardinal_direction(a-b);
}

void UAVDetail::moveTowardNextWaypoint() {
    if (!target_waypoints.size())
        return;  // return if no waypoints

    for (int i = 0; i < speed; i++) {
        loc = target_waypoints.front();
        target_waypoints.pop();
    }
}
