// Copyright 2016 Carrie Rebhuhn
#ifndef SIMULATION_SIMNE_H_
#define SIMULATION_SIMNE_H_

// C
#include <time.h>

// C++
#include <sstream>
#include <limits>

// Libraries
#include "ISimulator.h"
#include "Multiagent/MultiagentNE.h"
#include "Math/easymath.h"

class SimNE : public ISimulator {
 public:
    // SimNE(IDomainStateful* domain);
    SimNE(IDomainStateful* domain, MultiagentNE* MAS);
    virtual ~SimNE(void);

    static const int n_epochs = 100;
    static const int n_trials = 1;
    int* step;  // step counter for running the simulation

    virtual void runExperiment();
    virtual void epoch(int ep);
    //! Gets actions based on current state: OVERLOAD FOR TYPES
    virtual matrix2d getActions();
};
#endif  // SIMULATION_SIMNE_H_
