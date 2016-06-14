// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_SECTOR_H_
#define DOMAINS_UTM_SECTOR_H_

#include "UTMModesAndFiles.h"
#include "Link.h"
#include <vector>
#include <map>
#include "Fix.h"

class Sector {
 public:
    typedef std::pair<size_t,size_t> edge;
    // An area of airspace to control
    Sector(easymath::XY xy, size_t sectorIDset, std::vector<size_t> connections,
        std::vector<easymath::XY> dest_locs):
        xy(xy), ID(sectorIDset), connections(connections){}
    ~Sector() {}

    // Location properties
    const size_t ID;  // the identifier for this sector
    const std::vector<size_t> connections;
    const easymath::XY xy;  // sector center
    Fix* generation_pt;
};

#endif  // DOMAINS_UTM_SECTOR_H_
