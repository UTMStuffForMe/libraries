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
    std::printf("UAV %i created", ID);
}

void UAVDetail::planAbstractPath() {
    cur_sector = get_cur_sector();
    sectors_touched.insert(cur_sector);

    if (params->_search_type_mode == UTMModes::SearchDefinition::ASTAR){ //&& !committed_to_link) { // removed committed_to_link: different test applied
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

    //    list<XY> low_path = Planning::astar<GridGraph, easymath::XY>
     //       (lowGraph, loc, next_loc);

        // Add to target waypoints
    //clear(&target_waypoints);
//    for (XY i : low_path)
    //    target_waypoints.push(i);
    //target_waypoints.pop();  // removes current location from target

    // ~B
    if (next_sector != cur_sector) { // if not an internal link
            // Get the astar low-level path
        //XY next_loc = highGraph->getLocation(nsID);


        ///    vector<XY> low_path = lowGraph->astar(loc, next_loc); // called outside this
         //   std::reverse(low_path.begin(), low_path.end());
        list<XY> low_path = Planning::astar<GridGraph, easymath::XY>
            (lowGraph, loc, next_loc);

        // Add to target waypoints
        clear(&target_waypoints);
        for (XY i : low_path)
            target_waypoints.push_front(i);
        target_waypoints.pop_front();  // removes current location from target
        
        prev_targ.push_back(target_waypoints);
    }
    //next_link_ID = nextLinkID(); // need to rewrite -- next link?
}


void UAVDetail::moveTowardNextWaypoint() {
    // ~B
    prev_locs.push_back(loc);
    prev_secs.push_back(cur_sector);
//    prev_commits.push_back(committed_to_link);
    prev_mems.push_back(cur_sector);
    prev_cur_links.push_back(cur_link_ID);
    //prev_next_links.push_back(next_link_ID); /// replace (Carrie)
//    prev_high_path_prev.push_back(high_path_prev);  // Need to replace (Carrie)

//    next_sector_ID = nextSectorID(); // note: need to replace (Carrie)

//    if (!committed_to_link) // Not sure if best way to go about this
  //      return;

    //mem = next_sector_ID; // replace (Carrie)


    for (int i = 0; i < speed && !target_waypoints.empty(); i++) {
        loc = target_waypoints.front();
        target_waypoints.pop_front();
    }


    // TODO: UPDATE CURRENT SECTOR
    // TODO: UPDATE NEXT LINK (?)

        //next_link_ID = nextLinkID();

    reached_goal_sector = false;
    reached_next_sector = false;
    internal_link = false;
}

void UAVDetail::clearHistory() {
    prev_locs.clear();
    prev_mems.clear();
    prev_secs.clear();
    prev_commits.clear();
    prev_next_links.clear();
    prev_cur_links.clear();
    prev_high_path_prev.clear();
    next_sector_ID = -1;
    prev_targ.clear();

}

bool UAVDetail::at_boundary() {
    if (target_waypoints.size() <= 1)
        return false; // at destination, not boundary
    else {
        // peek at first two element of queue
        easymath::XY first_pt = target_waypoints.front();
        easymath::XY second_pt = *std::next(target_waypoints.begin());

        int m1 = lowGraph->get_membership(first_pt);
        int m2 = lowGraph->get_membership(second_pt);
        return (m1 != m2);
    }
}