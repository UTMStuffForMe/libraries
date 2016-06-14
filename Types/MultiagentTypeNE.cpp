// Copyright 2016 Carrie Rebhuhn
#include "MultiagentTypeNE.h"

MultiagentTypeNE::MultiagentTypeNE(int n_agents, NeuroEvoParameters* NE_params,
    TypeHandling type_mode, int n_types) {//:
    //MultiagentNE(n_agents, NE_params), type_mode(type_mode),
    //n_types(n_types) {    // need to change this...
    // USING SWITCH STATEMENT FOR OBJECT CREATION.
    // AFTER THIS POINT IN CODE, POLYMORPHISM USED.

    for (int i = 0; i < n_agents; i++) {
        switch (type_mode) {
        case MULTIMIND:
        {
            agents.push_back(new TypeNeuroEvo(NE_params, n_types));
            break;
        }
        case WEIGHTED:
        {
            // each type plays a part simultaneously
            agents.push_back(new NeuroEvoTypeWeighted(NE_params, n_types, NE_params->nInput));
            break;
        }
        case CROSSWEIGHTED:
        {
            // each type plays a part simultaneously
            agents.push_back(new NeuroEvoTypeCrossweighted(NE_params, n_types, 4));
            break;
        }
        case BLIND:
        {
            agents.push_back(new NeuroEvo(NE_params)) ;
            break;
        }
        default:
            printf("Error: invalid type handling mode.");
        }
    }
}


/*
std::vector<Action> MultiagentTypeNE::get_actions(std::vector<std::vector<State> > state) {
    std::vector<Action> actions(state.size());  // get an action vector for each agent
    for (size_t i = 0; i < agents.size(); i++) {
        actions[i] = agents[i]->get_action(state[i]);
    }
    return actions;
}
*/