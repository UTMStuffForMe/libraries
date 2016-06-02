// Copyright 2016 Carrie Rebhuhn
#ifndef MULTIAGENT_MULTIAGENTNE_H_
#define MULTIAGENT_MULTIAGENTNE_H_

#include "IMultiagentSystem.h"
#include "SingleAgent/NeuroEvo/NeuroEvo.h"

class MultiagentNE :
    public IMultiagentSystem {
 public:
    MultiagentNE(void);
    MultiagentNE(int n_agents, NeuroEvoParameters* NE_params);
    ~MultiagentNE(void);
    void generateNewMembers();
    virtual void selectSurvivors();
    virtual bool setNextPopMembers();

    NeuroEvoParameters* NE_params;
};
#endif  // MULTIAGENT_MULTIAGENTNE_H_
