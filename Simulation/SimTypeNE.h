// Copyright 2016 Carrie Rebhuhn

#ifndef SIMULATION_SIMTYPENE_H_
#define SIMULATION_SIMTYPENE_H_

#include "SimNE.h"
#include "Multiagent/MultiagentTypeNE.h"

class SimTypeNE : public SimNE {
 public:
    SimTypeNE(IDomainStateful *domain, MultiagentNE* MAS,
        MultiagentTypeNE::TypeHandling type_mode);
    ~SimTypeNE(void);

    NeuroEvoParameters* NE_params;
    MultiagentTypeNE::TypeHandling type_mode;
    virtual matrix2d getActions();
};
#endif  // SIMULATION_SIMTYPENE_H_
