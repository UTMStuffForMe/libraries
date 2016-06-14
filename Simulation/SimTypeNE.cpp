// Copyright 2016 Carrie Rebhuhn

#include "SimTypeNE.h"

using std::vector;

vector<Action> SimTypeNE::get_actions() {
    vector<vector<State> > S = domain->getTypeStates();  // [agent id][type id][state element
    return MAS->get_actions(S);
}

SimTypeNE::SimTypeNE(IDomainStateful *domain,
    MultiagentTypeNE* MAS, MultiagentTypeNE::TypeHandling type_mode) :
    SimNE(domain, MAS), type_mode(type_mode), MAS(MAS) {
}

SimTypeNE::~SimTypeNE(void) {
}
