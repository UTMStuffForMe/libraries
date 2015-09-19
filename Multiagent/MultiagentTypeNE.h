#pragma once
#include "MultiagentNE.h"
#include "../SingleAgent/NeuroEvo/TypeNeuroEvo.h"
#include "../SingleAgent/NeuroEvo/NeuroEvo.h"
#include "../SingleAgent/NeuroEvo/NeuroEvoTypeWeighted.h"
#include "../SingleAgent/NeuroEvo/NeuroEvoTypeCrossweighted.h"

// Container for collection of 'Type Neuro Evo' agents
class MultiagentTypeNE: public MultiagentNE
{
public:
	const enum TypeHandling{BLIND, WEIGHTED, CROSSWEIGHTED, MULTIMIND, NMODES}; // options for handling different types
	TypeHandling type_mode;
	int n_types;

	MultiagentTypeNE(void){};

	MultiagentTypeNE(int n_agents, NeuroEvoParameters* NE_params, TypeHandling type_mode, int n_types);
	~MultiagentTypeNE(void);

	//void initializeWithStereotypes(std::vector<std::vector<NeuroEvo*> > stereotypes, std::vector<int> agent_types);
	matrix2d getActions(matrix3d state);

	virtual bool setNextPopMembers(){
		// Kind of hacky; select the next member and return true if not at the end
		// Specific to Evo

		std::vector<bool> is_another_member(agents.size(),false);
		for (unsigned int i=0; i<agents.size(); i++){
			if (type_mode==MULTIMIND) is_another_member[i] = ((TypeNeuroEvo*)agents[i])->selectNewMemberAll();
			else if (type_mode==WEIGHTED || type_mode==CROSSWEIGHTED || type_mode==BLIND) is_another_member[i] = ((NeuroEvo*)agents[i])->selectNewMember();
		}
		for (bool a: is_another_member){
			if (!a){
				return false;
			}
		}

		return true;
	}

	virtual void selectSurvivors(){
	// Specific to Evo: select survivors
	for (IAgent* a: agents){
		if (type_mode==MULTIMIND){
		((TypeNeuroEvo*)a)->selectSurvivorsAll();
		} else if (type_mode==WEIGHTED || type_mode==CROSSWEIGHTED|| type_mode==BLIND){
			((NeuroEvo*)a)->selectSurvivors();
		}
	}
}
};