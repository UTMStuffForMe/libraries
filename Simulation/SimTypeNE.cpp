// Copyright 2016 Carrie Rebhuhn

#include "SimTypeNE.h"

matrix2d SimTypeNE::getActions() {
    matrix3d S = domain->getTypeStates();  // [agent id][type id][state element
    return MAS->getActions(S);
}

SimTypeNE::SimTypeNE(IDomainStateful *domain,
    MultiagentTypeNE* MAS, MultiagentTypeNE::TypeHandling type_mode) :
    SimNE(domain, MAS), type_mode(type_mode), MAS(MAS) {
}

SimTypeNE::~SimTypeNE(void) {
}
