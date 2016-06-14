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
    vector<XY> dest_locs, UTMModes* params) :
    highGraph(highGraph), destination_locs(dest_locs), ID(ID_set),
    loc(loc), params(params) {
}

bool Fix::should_generate_UAV(int step) {
    if (params->_traffic_mode == UTMModes::TrafficMode::PROBABILISTIC) {
        double pnum = rand(0, 1);
        double pgen = params->get_p_gen();
        if (pnum > pgen)
            return false;
        else
            return true;
    } else {
        // deterministic
        int genrate = params->get_gen_rate();
        if (step%genrate != 0)
            return false;
        else
            return true;
    }
}

UAV* Fix::generate_UAV(int step) {
    // Creates a new UAV in the world
    if (should_generate_UAV(step))
        return generate_UAV();
    else
        return NULL;
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

    UAV* u = new UAV(highGraph->at(type_id_set)->get_membership(loc),
        highGraph->at(type_id_set)->get_membership(end_loc),
        static_cast<UTMModes::UAVType>(type_id_set),
        highGraph->at(type_id_set), params);
    return u;
}