// Copyright 2016 Carrie Rebhuhn
#include "MultiagentTypeNE.h"

MultiagentTypeNE::MultiagentTypeNE(int n_agents, NeuroEvoParameters* NE_params,
    TypeHandling type_mode, int n_types) :
    MultiagentNE(n_agents, NE_params), type_mode(type_mode),
    n_types(n_types) {
    // USING SWITCH STATEMENT FOR OBJECT CREATION.
    // AFTER THIS POINT IN CODE, POLYMORPHISM USED.

    for (IAgent* a : agents) {
        switch (type_mode) {
        case MULTIMIND:
        {
            *a = TypeNeuroEvo(NE_params, n_types);
            break;
        }
        case WEIGHTED:
        {
            // each type plays a part simultaneously
            *a = NeuroEvoTypeWeighted(NE_params, n_types, NE_params->nInput);
            break;
        }
        case CROSSWEIGHTED:
        {
            // each type plays a part simultaneously
            *a = NeuroEvoTypeCrossweighted(NE_params, n_types, 4);
            break;
        }
        case BLIND:
        {
            *a = NeuroEvo(NE_params);
        }
        default:
            printf("Error: invalid type handling mode.");
        }
    }
}

MultiagentTypeNE::~MultiagentTypeNE(void) {
}

matrix2d MultiagentTypeNE::getActions(matrix3d state) {
    matrix2d actions(state.size());  // get an action vector for each agent
    for (size_t i = 0; i < agents.size(); i++) {
        actions[i] = agents[i]->getAction(state[i]);
    }
    return actions;
}
