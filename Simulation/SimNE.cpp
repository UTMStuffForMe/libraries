// Copyright 2016 Carrie Rebhuhn
#include "SimNE.h"

#include "float.h"

SimNE::SimNE(IDomainStateful* domain, MultiagentNE* MAS) :
    ISimulator(domain, MAS), step(new size_t(0)) {
    domain->synch_step(step);
}

SimNE::~SimNE(void) {
    delete step;
}

void SimNE::runExperiment() {
    for (int ep = 0; ep < n_epochs; ep++) {
        time_t epoch_start = time(NULL);
        this->epoch(ep);
        time_t epoch_end = time(NULL);
        time_t epoch_time = epoch_end - epoch_start;
        time_t run_time_left = (time_t(n_epochs - ep))*epoch_time;
        time_t run_end_time = epoch_end + run_time_left;

        char end_clock_time[26];
#ifdef _WIN32
        ctime_s(end_clock_time, sizeof(end_clock_time), &run_end_time);
#endif

         printf("Epoch %i took %i seconds.\n",ep,size_t(epoch_time));
         std::cout << "Estimated run end time: " << end_clock_time << std::endl;
    }
}

void SimNE::runExperimentDifference() {
    for (int ep = 0; ep < n_epochs; ep++) {
        time_t epoch_start = time(NULL);
        this->epoch_difference(ep);
        time_t epoch_end = time(NULL);
        time_t epoch_time = epoch_end - epoch_start;
        time_t run_time_left = (time_t(n_epochs - ep))*epoch_time;
        time_t run_end_time = epoch_end + run_time_left;

        char end_clock_time[26];
#ifdef _WIN32
        ctime_s(end_clock_time, sizeof(end_clock_time), &run_end_time);
#endif

        printf("Epoch %i took %i seconds.\n", ep, size_t(epoch_time));
        std::cout << "Estimated run end time: " << end_clock_time << std::endl;
    }
}

void SimNE::run_simulation(bool log, int neural_net, int suppressed_agent) {
    for ((*step) = 0; (*step) < domain->n_steps; (*step)++) {
        matrix2d A = this->getActions();
        if (suppressed_agent >= 0) {
            A[suppressed_agent] = easymath::zeros(A[suppressed_agent].size());
        }
        domain->simulateStep(A, neural_net);
        
        if (log)
            domain->logStep();
    }
}

void SimNE::epoch_difference(int ep) {
    printf("Epoch %i", ep);
    SimNE::accounting accounts = SimNE::accounting();
    reinterpret_cast<MultiagentNE*>(MAS)->generateNewMembers();
    int neural_net_ID = 0;
    do {
        run_simulation(neural_net_ID, false);
        double G = domain->getPerformance()[0];
        domain->reset();
        matrix1d D(MAS->agents.size(), 0.0);

        accounts.update(domain->getRewards(), domain->getPerformance());

        for (size_t i = 0; i < MAS->agents.size(); i++) {
            run_simulation(false, neural_net_ID, i);  // agent i suppressed
            double Gc = domain->getPerformance()[0];

            D[i] = G - Gc;
            domain->reset();
        }
        neural_net_ID++;
        MAS->updatePolicyValues(D);
    } while (reinterpret_cast<MultiagentNE*>(MAS)->setNextPopMembers());
    reinterpret_cast<MultiagentNE*>(MAS)->selectSurvivors();

    reward_log.push_back(accounts.best_run);
    metric_log.push_back(accounts.best_run_performance);
}

void SimNE::epoch(int ep) {
    bool log = (ep == 0 || ep == n_epochs - 1) ? true : false;

    reinterpret_cast<MultiagentNE*>(MAS)->generateNewMembers();
    SimNE::accounting accounts = SimNE::accounting();

    int n = 0; // neural net number
    do {
        // Gets the g
        run_simulation(log, n++);
        matrix1d R = domain->getRewards();
        matrix1d perf = domain->getPerformance();

        accounts.update(R, perf);

        domain->reset();
        MAS->updatePolicyValues(R);
    } while (reinterpret_cast<MultiagentNE*>(MAS)->setNextPopMembers());
    reinterpret_cast<MultiagentNE*>(MAS)->selectSurvivors();


    reward_log.push_back(accounts.best_run);
    metric_log.push_back(accounts.best_run_performance);
    if (ep == 0)
        domain->exportStepsOfTeam(accounts.best_perf_idx, "untrained");
    if (ep == n_epochs - 1)
        domain->exportStepsOfTeam(accounts.best_perf_idx, "trained");
}

matrix2d SimNE::getActions() {
    matrix2d S = domain->getStates();
    return MAS->getActions(S);
}
