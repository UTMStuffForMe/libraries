#include "FixDetail.h"

using easymath::XY;

bool FixDetail::atDestinationFix(const UAVDetail &u) {
    bool has_plan = u.target_waypoints.size() > 0;
    bool is_next = u.target_waypoints.front() == loc;
    bool is_close = euclidean_distance(u.loc, loc) < approach_threshold;
    bool is_goal = u.end_loc == loc;

    return has_plan && is_next && is_close && is_goal;
}

UAVDetail* FixDetail::generate_UAV() {
    static int calls = 0;
    XY end_loc;
    if (ID == 0)
        end_loc = destination_locs.back();
    else
        end_loc = destination_locs.at(ID - 1);  // go to previous

    int type_id_set = calls%params->get_n_types();
    UAVDetail* u = new UAVDetail(loc, end_loc,
        static_cast<UTMModes::UAVType>(type_id_set),
        highGraph, linkIDs, params, lowGraph);
    u->planAbstractPath();
    return u;
}
