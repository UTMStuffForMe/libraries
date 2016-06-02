// Copyright 2016 Carrie Rebhuhn
#include "SimNE.h"

#include "float.h"

SimNE::SimNE(IDomainStateful* domain, MultiagentNE* MAS) :
    ISimulator(domain, MAS), step(new int(0)) {
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

void SimNE::epoch(int ep) {
    bool log = (ep == 0 || ep == n_epochs - 1) ? true : false;

    reinterpret_cast<MultiagentNE*>(MAS)->generateNewMembers();
    double best_run = -DBL_MAX;
    double best_run_performance = -DBL_MAX;

    int n = 0;  // neural net number (for output file name)

    int best_perf_idx = 0;  // the team that performed the best

    do {
        matrix2d Rtrials;   // Trial average reward
        for (int t = 0; t < n_trials; t++) {
            for ((*step) = 0; (*step) < domain->n_steps; (*step)++) {
                // must be called by 'this' in order to access potential child
                // class overload
                matrix2d A = this->getActions();
                domain->simulateStep(A);

                if (log)
                    // Log positions of UAVs
                    domain->logStep();
            }

            matrix1d R = domain->getRewards();
            matrix1d perf = domain->getPerformance();

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

            std::ostringstream epi, ni, ti;
            epi << ep;
            ni << n;
            ti << t;

            Rtrials.push_back(R);


            domain->reset();
        }
        // based on the trials...
        MAS->updatePolicyValues(easymath::mean2(Rtrials));

        n++;
    } while (reinterpret_cast<MultiagentNE*>(MAS)->setNextPopMembers());
    reinterpret_cast<MultiagentNE*>(MAS)->selectSurvivors();

    reward_log.push_back(best_run);
    metric_log.push_back(best_run_performance);
    if (ep == 0)
        domain->exportStepsOfTeam(best_perf_idx, "untrained");
    if (ep == n_epochs - 1)
        domain->exportStepsOfTeam(best_perf_idx, "trained");
}

matrix2d SimNE::getActions() {
    matrix2d S = domain->getStates();
    return MAS->getActions(S);
}
