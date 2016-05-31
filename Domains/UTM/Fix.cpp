// Copyright 2016 Carrie Rebhuhn
#include "Fix.h"
#include <vector>
#include <list>
#include <map>

using std::vector;
using std::list;
using std::map;
using easymath::XY;
using easymath::rand;

Fix::Fix(XY loc, int ID_set, MultiGraph<LinkGraph>* highGraph,
    vector<XY> dest_locs,
    UTMModes* params,
    map<edge, int> *linkIDs) :
    highGraph(highGraph), destination_locs(dest_locs), ID(ID_set),
    loc(loc), params(params), linkIDs(linkIDs) {
}


list<UAV* > Fix::generateTraffic(int step) {
    // Creates a new UAV in the world
    std::list<UAV* > newTraffic;

    switch (params->_traffic_mode) {
    case UTMModes::TrafficMode::PROBABILISTIC:
        if (rand(0, 1) > params->get_p_gen())  // Doesn't generate a UAV
            return newTraffic;
        break;
    case UTMModes::TrafficMode::DETERMINISTIC:
        if (step%params->get_gen_rate() != 0)  // Doesn't generate a UAV
            return newTraffic;
        break;
    default:
        return newTraffic;
    }

    UAV* u = generate_UAV();
    newTraffic.push_back(u);

    return newTraffic;
}

UAV* Fix::generate_UAV() {
    static int calls = 0;
    XY end_loc;
    if (ID == 0)
        end_loc = destination_locs.back();
    else
        end_loc = destination_locs.at(ID - 1);  // go to previous

    // Creates an equal number of each type;
    int type_id_set = calls%params->get_n_types();
    calls++;
    UAV* u = new UAV(highGraph->at()->get_membership(loc),
        highGraph->at()->get_membership(end_loc),
        static_cast<UTMModes::UAVType>(type_id_set),
        highGraph, linkIDs, params);
    u->planAbstractPath();
    return u;
}