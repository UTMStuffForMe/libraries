#pragma once
#include "Domains/UTM/Sector.h"
#include "FixDetail.h"

class SectorDetail : public Sector {
public:
    SectorDetail(easymath::XY xy, int sectorIDset,
        std::vector<int> connections, std::vector<easymath::XY> dest_locs,
        MultiGraph<LinkGraph>* highGraph, MultiGraph<GridGraph>* lowGraph,
        UTMModes* params, std::list<UAVDetail*>* UAVs_done) :
        Sector(xy, sectorIDset, connections, dest_locs) {

        FixDetail* f = new FixDetail(xy, sectorIDset, highGraph, lowGraph, dest_locs, params);
        f->UAVs_stationed = UAVs_done;
        generation_pt = f;
    }

    FixDetail *generation_pt;
};
