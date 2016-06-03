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


class UTMDomainAbstract :
    public IDomainStateful {
 public:
    typedef std::pair<int, int> edge;
    explicit UTMDomainAbstract(UTMModes* params);
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
    std::map<edge, int> *linkIDs;


    // Traffic
    std::list<UAV*> UAVs;
    void getNewUAVTraffic();
    void absorbUAVTraffic();


    // Graphs/search objects
    MultiGraph<LinkGraph> *highGraph;
    RAGS* rags_map;

    // Base function overloads
    matrix2d getStates();
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
    void try_to_move(std::vector<UAV*> * eligible_to_move);

    // this has moved or something?
    // void move_UAV_to_link(UAV* u, Link* cur_link, Link* new_link);
    // handles motion of the UAV in the simulation, also includes logging

 private:
    // records number of UAVs at each sector at current time step
    matrix1d numUAVsAtSector;
};
#endif  // DOMAINS_UTM_UTMDOMAINABSTRACT_H_
