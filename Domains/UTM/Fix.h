// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_FIX_H_
#define DOMAINS_UTM_FIX_H_

// STL includes
#include <list>
#include <vector>
#include <utility>

// Library includes
#include "../../Planning/MultiGraph.h"
#include "UAV.h"

class Fix {
 public:
    typedef std::pair<int, int> edge;
    Fix(easymath::XY loc, int ID, MultiGraph<LinkGraph>* highGraph,
        std::vector<easymath::XY> dest_locs,
        UTMModes* params);


    ~Fix() {}
    UTMModes* params;
    std::list<UAV*> generateTraffic(int step);
    int ID;
    easymath::XY loc;
    virtual UAV* generate_UAV();

    MultiGraph<LinkGraph>* highGraph;
    std::vector<easymath::XY> destination_locs;
};
#endif  // DOMAINS_UTM_FIX_H_

