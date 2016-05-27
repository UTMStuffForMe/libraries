// Copyright 2016 Carrie Rebhuhn

#include "UAV.h"

#include <list>
#include <map>

using std::list;

UAV::UAV(int start_mem, int mem_end, UTMModes::UAVType my_type,
    MultiGraph<LinkGraph> highGraph, std::map<edge, int>* linkIDs, UTMModes* params) :
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

int UAV::nextLinkID() {
    if (nextSectorID(1) == nextSectorID(2)) {
        return curLinkID();
    } else {
        edge link(nextSectorID(1), nextSectorID(2));
        return linkIDs->at(link);
    }
}

std::list<size_t> UAV::getBestPath() {
    return Planning::astar<LinkGraph,size_t>(highGraph(type_ID), mem, mem_end);
}

void UAV::planAbstractPath() {
    if (!on_internal_link) links_touched.insert(cur_link_ID);
    sectors_touched.insert(curSectorID());

    list<size_t> high_path;
    int cur_s = curSectorID();
    int end_s = endSectorID();
    if (params->_search_type_mode == UTMModes::SearchDefinition::ASTAR) {
        high_path = Planning::astar<LinkGraph,size_t>(highGraph(type_ID), cur_s, end_s);
    } else {
        //high_path = highGraph(type_ID)->rags(cur_s, end_s);
    }
    if (high_path.empty()) {
        printf("Path not found!");
        system("pause");
    }
    if (high_path_prev != high_path) {
        pathChanged = true;
        high_path_prev = high_path;
    } else {
        pathChanged = false;
    }

    next_link_ID = nextLinkID();
}

int UAV::getDirection() {
    // Identifies whether traveling in one of four cardinal directions
    return highGraph()->get_direction(mem, nextSectorID());
}