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
    size_t* step;  // step counter for running the simulation

    virtual void runExperiment();
    virtual void epoch(int ep);

    void epoch_difference(int ep);
    void run_simulation(bool log, int neural_net_ID, int suppressed_agent=-1);

    //! Gets actions based on current state: OVERLOAD FOR TYPES
    virtual matrix2d getActions();
    void runExperimentDifference();
    struct accounting{
        accounting() {
            best_run = -DBL_MAX;
            best_run_performance = -DBL_MAX;
            n = 0;
            best_perf_idx = 0;
        }

        void update(const matrix1d &R, const matrix1d &perf) {
            double avg_G = easymath::mean(R);
            double avg_perf = easymath::mean(perf);

            if (avg_G > best_run) {
                best_run = avg_G;
            }
            if (avg_perf > best_run_performance) {
                best_run_performance = avg_perf;
                best_perf_idx = n;
            }

            printf("NN#%i, %f, %f, %f\n", n, best_run_performance,
                best_run, avg_perf);
        }
        int n, best_perf_idx;
        double best_run, best_run_performance;
    };
};
#endif  // SIMULATION_SIMNE_H_
