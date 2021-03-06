// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_UTMDOMAINABSTRACT_H_
#define DOMAINS_UTM_UTMDOMAINABSTRACT_H_
#include <memory>
#include <string>
#include <map>
#include <utility>
#include <list>
#include <vector>

#include "Domains/IDomainStateful.h"
#include "Math/easymath.h"
#include "Planning/MultiGraph.h"
#include "Planning/RAGS.h"

#include "UAV.h"
#include "Sector.h"
#include "Link.h"
#include "Fix.h"
#include "FileIO/FileOut.h"
#include "FileIO/FileIn.h"
#include "SectorAgentManager.h"

#define UTM_ABSTRACT_VERBOSE

class UTMDomainAbstract :
    public IDomainStateful {
public:
    typedef std::pair<size_t, size_t> edge;
    explicit UTMDomainAbstract(UTMModes* params);
    explicit UTMDomainAbstract(UTMModes* params, bool only_abstract);
    ~UTMDomainAbstract(void);

    // virtual void initialize(UTMModes* params);

    virtual void synch_step(size_t* step_set) {
        step = step_set;
        agents->steps = step_set;
        printf("set the step\n");
    }

    UTMModes* params;
    UTMFileNames* filehandler;


    // Agents
    IAgentManager* agents;

    // Moving parts
    std::vector<Sector*> sectors;
    std::vector<Link*> links;
    std::vector<Fix*> fixes;
    std::map<edge, size_t> *linkIDs;


    // Traffic
    std::list<UAV*> UAVs;
    virtual void getNewUAVTraffic();
    virtual void absorbUAVTraffic();


    // Graphs/search objects
    MultiGraph<LinkGraph> *highGraph;
    RAGS* rags_map;

    // Base function overloads
    matrix2d get_states();
    matrix3d getTypeStates();
    void simulateStep(matrix2d agent_actions);
    void logStep();
    // The number of UAVs on each link, [step][linkID]

    matrix2d linkUAVs;
    // The number of UAVs waiting at a sector, [step][sectorID]
    matrix2d sectorUAVs;

    void exportStepsOfTeam(int team, std::string suffix);
    std::string createExperimentDirectory();

    void exportSectorLocations(int fileID);

    // Different from children
    virtual matrix1d getPerformance();
    virtual matrix1d getRewards();
    virtual void incrementUAVPath();
    virtual void detectConflicts();

    virtual void getPathPlans();
    virtual void getPathPlans(const std::list<UAV*> &new_UAVs);
    virtual void reset();


    //! Moves all it can in the list.
    // Those eligible to move but who are blocked are left after the function.
    virtual void try_to_move(std::vector<UAV*> * eligible_to_move);

protected:
    // records number of UAVs at each sector at current time step
    matrix1d numUAVsAtSector;

public:
    //~B
        // The UAVs that have reached their goals (mapping: sector -> UAVs that are "standing by")
    std::map<int, std::list<UAV*> > UAVs_done;
    std::map<int, std::list<int> > incoming_links;


    size_t next_link(UAV* u) {
        return linkIDs->at(edge(u->get_cur_sector(), u->get_next_sector()));
    }
};
#endif  // DOMAINS_UTM_UTMDOMAINABSTRACT_H_
