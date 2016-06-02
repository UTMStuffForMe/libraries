// Copyright 2016 Carrie Rebhuhn

#include "SimTypeNE.h"

matrix2d SimTypeNE::getActions() {
    matrix3d S = domain->getTypeStates();  // [agent id][type id][state element
    return static_cast<MultiagentTypeNE*>(MAS)->getActions(S);
}

/*
SimTypeNE::SimTypeNE(IDomainStateful* domain, MultiagentTypeNE::TypeHandling type_mode):
    SimNE(domain),
    type_mode(type_mode)//,
    //NE_params(new NeuroEvoParameters(domain->n_state_elements,domain->n_control_elements))
{
    //MAS = new MultiagentTypeNE(domain->n_agents,NE_params,type_mode,domain->n_types);
}

removed -- this can cause memory leaks
*/

SimTypeNE::SimTypeNE(IDomainStateful *domain,
    MultiagentNE* MAS, MultiagentTypeNE::TypeHandling type_mode) :
    SimNE(domain, MAS), type_mode(type_mode) {
}

SimTypeNE::~SimTypeNE(void) {
}
