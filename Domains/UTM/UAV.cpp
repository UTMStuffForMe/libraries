// Copyright 2016 Carrie Rebhuhn

#include "UAV.h"

#include <list>
#include <map>

using std::list;

UAV::UAV(int start_sector, int end_sector_set, UTMModes::UAVType my_type,
    LinkGraph* highGraph, UTMModes* params) :
    highGraph(highGraph), cur_sector(start_sector), end_sector(end_sector_set),
    type_ID(size_t(my_type)), speed(1.0), params(params),
    times_reached_goal(0){
    
    static int calls = 0;
    ID = calls++;

    // Get initial plan and update
    planAbstractPath();
}

void UAV::planAbstractPath() {
    sectors_touched.insert(cur_sector);

    if (params->_search_type_mode == UTMModes::SearchDefinition::ASTAR) {
        high_path = Planning::astar<LinkGraph,size_t>(highGraph, cur_sector, end_sector);
    } else {
        //high_path = highGraph->at(type_ID)->rags(cur_s, end_s);
    }

    if (high_path.empty()) {
        printf("Path not found!");
        system("pause");
    } 
    
    // Set variables
    next_sector = get_next_sector();
}