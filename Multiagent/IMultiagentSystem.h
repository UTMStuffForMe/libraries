// Copyright 2016 Carrie Rebhuhn
#ifndef MULTIAGENT_IMULTIAGENTSYSTEM_H_
#define MULTIAGENT_IMULTIAGENTSYSTEM_H_


// STL includes
#include <vector>

// Library includes
#include "SingleAgent/IAgent.h"

//! Agents container
class IMultiagentSystem {
 public:
    IMultiagentSystem(void);
    virtual ~IMultiagentSystem(void);

    // Set of agents in the system (set externally)
    std::vector<IAgent*> agents;

    matrix2d getActions(matrix2d S);
    void updatePolicyValues(matrix1d R);
};
#endif  // MULTIAGENT_IMULTIAGENTSYSTEM_H_
