// Copyright 2016 Carrie Rebhuhn

#ifndef DOMAINS_UTM_UTMDOMAINDETAIL_H_
#define DOMAINS_UTM_UTMDOMAINDETAIL_H_

// STL includes
#include <utility>
#include <algorithm>
#include <vector>
#include <list>
#include <string>

// Library includes
#include "../../../FileIO/FileIn.h"
#include "../UTMDomainAbstract.h"
#include "FixDetail.h"


class UTMDomainDetail :
    public UTMDomainAbstract {
 public:
    explicit UTMDomainDetail(UTMModes* params_set);
    virtual ~UTMDomainDetail(void);

    // Base function overloads
    virtual matrix1d getRewards();
    virtual matrix1d getPerformance();
    virtual void getPathPlans();  // note: when is this event?
    virtual void getPathPlans(const std::list<UAV*> &new_UAVs);
    virtual void exportLog(std::string fid, double G);
    virtual void detectConflicts();
    virtual void incrementUAVPath();
    virtual void reset();

    // maps/Graph
    MultiGraph<GridGraph>* lowGraph;

    void addConflict(UAV* u1, UAV* u2) {
        agents->metrics.at(u1->get_cur_sector()).local[u1->get_type()] += 0.5;
        agents->metrics.at(u2->get_cur_sector()).local[u2->get_type()] += 0.5;
    }
    size_t getSector(easymath::XY p);

    // UAV motion tracking
    void logUAVLocations();

    //! UAVLocation is nUAVs*2 x nSteps long, with the first dimension being
    //! twice as long because there are x- and y-values
    matrix2d UAVLocations;
    void exportUAVLocations(int fileID);
};
#endif  // DOMAINS_UTM_UTMDOMAINDETAIL_H_
