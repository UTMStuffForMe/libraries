#pragma once

#include "Domains/UTM/Fix.h"
#include "UAVDetail.h"

class FixDetail : public Fix {
public:
    FixDetail(easymath::XY loc, size_t ID, MultiGraph<LinkGraph>* highGraph,
        MultiGraph<GridGraph>* lowGraph, std::vector<easymath::XY> dest_locs, UTMModes* params) :
        Fix(loc, ID, highGraph, dest_locs, params),
        lowGraph(lowGraph), approach_threshold(params->get_dist_thresh()),
        conflict_threshold(params->get_conflict_thresh())
    {};

    virtual ~FixDetail() {}
    MultiGraph<GridGraph>* lowGraph;

    //! Calls a conditional, then creates UAV in the world
    virtual UAVDetail* generate_UAV(int step);


    virtual bool atDestinationFix(const UAVDetail &u);
    std::list<UAVDetail*> * UAVs_stationed;

private:

    //! Creates a new UAV in the world
    virtual UAVDetail* generate_UAV();
    double approach_threshold;
    double conflict_threshold;
};