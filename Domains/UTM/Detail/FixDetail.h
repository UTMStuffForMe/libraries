#pragma once

#include "../Fix.h"
#include "UAVDetail.h"

class FixDetail : public Fix {
public:
    FixDetail(easymath::XY loc, int ID, MultiGraph<LinkGraph> highGraph,
        MultiGraph<GridGraph> lowGraph,
        std::vector<easymath::XY> dest_locs,
        UTMModes* params, std::map<std::pair<int, int>, int> *linkIDs) :
        Fix(loc, ID, highGraph,
            dest_locs,
            params, linkIDs),
        lowGraph(lowGraph), approach_threshold(params->get_dist_thresh()),
        conflict_threshold(params->get_conflict_thresh())

    {};
    ~FixDetail() {}
    MultiGraph<GridGraph> lowGraph;
    virtual UAVDetail* generate_UAV();
    virtual bool atDestinationFix(const UAVDetail &u);


    // Approach
    double approach_threshold;
    double conflict_threshold;
};