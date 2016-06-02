// Copyright 2016 Carrie Rebhuhn
#ifndef SINGLEAGENT_IAGENT_H_
#define SINGLEAGENT_IAGENT_H_

#include "Math/easymath.h"

class IAgent {
 public:
    IAgent(void);
    virtual ~IAgent(void) {}

    // Gets an action given a state
    virtual matrix1d getAction(matrix1d state) = 0;

    // Gets an action given a 2d state
    virtual matrix1d getAction(matrix2d state) = 0;

    // Update current policy given a reward
    virtual void updatePolicyValues(double R) = 0;
};

#endif  // SINGLEAGENT_IAGENT_H_
