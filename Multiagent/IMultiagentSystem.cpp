// Copyright 2016 Carrie Rebhuhn
#include "IMultiagentSystem.h"

IMultiagentSystem::IMultiagentSystem(void) {
}


IMultiagentSystem::~IMultiagentSystem(void) {
}

matrix2d IMultiagentSystem::getActions(matrix2d S) {
    if (!S.size()) {
        printf("Zero state size!");
        system("pause");
    }
    matrix2d A(agents.size());
    // get all actions, given a list of states
    for (size_t i = 0; i < agents.size(); i++) {
        A[i] = agents[i]->getAction(S[i]);
    }
    return A;
}
