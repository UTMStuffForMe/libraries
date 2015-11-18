#include "SimNE.h"

SimNE::SimNE(IDomainStateful* domain):
	ISimulator(domain, new MultiagentNE(domain->n_agents, new NeuroEvoParameters(domain->n_state_elements,domain->n_control_elements)))
{
}

// FOR DEBUGGING
SimNE::SimNE(IDomainStateful* domain, MultiagentNE* MAS):
	ISimulator(domain, MAS)
{
}

SimNE::~SimNE(void)
{
	delete ((MultiagentNE*)MAS)->NE_params;
	delete MAS;
}

void SimNE::runExperiment(){
	for (int ep=0; ep<n_epochs; ep++){
		printf("Epoch %i\n",ep);
		//printf(".");
		this->epoch(ep);
	}
}

void SimNE::epoch(int ep){
	((MultiagentNE*)MAS)->generateNewMembers();
	double best_run = -DBL_MAX;
	double best_run_performance = -DBL_MAX;

	int n=0; // neural net number (for output file name)

	do{
		matrix2d Rtrials; // Trial average reward
		for (int t=0; t<n_trials; t++){
			clock_t tref = clock();
			for (int s=0; s<domain->n_steps; s++){
				//printf("Step %i\n",s);
				matrix2d A = this->getActions(); // must be called by 'this' in order to access potential child class overload
				domain->simulateStep(A);
				domain->logStep(s);
			}
			t= clock();
			printf("t=%f\n",float(t-tref)/CLOCKS_PER_SEC);
			tref=t;

			matrix1d R = domain->getRewards();
			matrix1d perf = domain->getPerformance();

			double avg_G = mean(R);
			double avg_perf = mean(perf);

			// note: things here get specific to the utm domain
			if (avg_G>best_run) { // NOTE: STILL SORTED BY BEST RUN, NOT BEST PERFORMANCE!
				best_run = avg_G;
				/*best_run_performance = avg_perf; // average of the performance metrics
				if (ep==0){
					domain->exportLog("stat_results/conflict_map-0-", ep); // blatant abuse of exportlog
				}
				if (ep==99){
					domain->exportLog("stat_results/conflict_map-99-",ep);
				}*/
			}
			if (avg_perf>best_run_performance){
				best_run_performance = avg_perf;
			}

			printf("NN#%i, %f, %f\n",n, best_run_performance, best_run);
			//printf(".");
			ostringstream epi,ni,ti;
			epi << ep;
			ni << n;
			ti << t;
			
			Rtrials.push_back(R);
			//system("pause");
			

			domain->reset();
		}
		MAS->updatePolicyValues(mean2(Rtrials)); // based on the trials...

		n++;
	} while (((MultiagentNE*)MAS)->setNextPopMembers()); // this also takes care of reset functions
	((MultiagentNE*)MAS)->selectSurvivors();

	reward_log.push_back(best_run);
	metric_log.push_back(best_run_performance);
}

matrix2d SimNE::getActions(){
	matrix2d S = domain->getStates();
	return MAS->getActions(S);
}