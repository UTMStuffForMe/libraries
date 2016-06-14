// Copyright 2016 Carrie Rebhuhn
#include "TypeNeuroEvo.h"


TypeNeuroEvo::TypeNeuroEvo(NeuroEvoParameters* NEParams, size_t nTypes) :
    NETypes(std::vector<NeuroEvo*>(nTypes)),
    xi(matrix1d(nTypes, 0.0)) {
    for (NeuroEvo* ne : NETypes) {
        ne = new NeuroEvo(NEParams);
    }
}
void TypeNeuroEvo::deep_copyNETypes(const std::vector<NeuroEvo*> &NETypesSet) {
    // Creates new pointer addresses for the neuro evo instances
    // Calls functions inside neuro evo to create new neural net pointers
    // deletes original pointers

    for (NeuroEvo* ne : NETypes) {
        delete ne;
    }

    NETypes = std::vector<NeuroEvo*>(NETypesSet.size());
    for (size_t i = 0; i < NETypesSet.size(); i++) {
        NETypes[i] = new NeuroEvo();
        NETypes[i]->deep_copy(*NETypesSet[i]);
        NETypes[i]->pop_member_active = NETypes[i]->population.begin();
    }
}

void TypeNeuroEvo::generate_new_members() {
    for (NeuroEvo* ne : NETypes) {
        ne->generate_new_members();
    }
}

bool TypeNeuroEvo::select_new_memberAll() {
    // note; only checks the last
    bool selected = false;
    for (NeuroEvo* ne : NETypes) {
        selected = ne->select_new_member();
    }
    return selected;
}

matrix1d TypeNeuroEvo::getBestMemberValAll() {
    matrix1d memberVals = matrix1d(NETypes.size());
    for (size_t i = 0; i < NETypes.size(); i++) {
        memberVals[i] = NETypes[i]->getBestMemberVal();
    }
    return memberVals;
}

void TypeNeuroEvo::select_survivorsAll() {
    for (NeuroEvo* ne : NETypes) {
        ne->select_survivors();
    }
}

matrix1d TypeNeuroEvo::get_action(matrix1d state) {
    printf("get_action being called in multimind setting. Need ");
    printf("neighbor_type identification, or else this will not work. ");
    printf("Debug before continuing.");
    std::system("pause");
    return matrix1d();
}


matrix1d TypeNeuroEvo::get_action(matrix1d state, size_t neighbor_type) {
    xi[neighbor_type]++;
    return NETypes[neighbor_type]->get_action(state);
}


matrix1d TypeNeuroEvo::get_action(matrix2d state) {
    // vote among all TYPES for an action
    matrix1d action_sum = get_action(state[0], 0);

    // starts at 1: initialized by 0
    for (size_t j = 1; j < state.size(); j++) {
        // specifies which NN to use
        matrix1d action_sum_temp = get_action(state[j], j);
        for (size_t k = 0; k < action_sum.size(); k++) {
            action_sum[k] += action_sum_temp[k];
        }
    }
    for (double &a : action_sum) {
        // normalize (magnitude unbounded for voting)
        a /= static_cast<double>(state.size());
    }
    return action_sum;
}

void TypeNeuroEvo::update_policy_values(double R) {
    // Add together xi values, for averaging
    double sumXi = easymath::sum(xi);
    for (size_t i = 0; i < NETypes.size(); i++) {
        // scaled proportional to other member values
        double xi_i = xi[i] / sumXi;
        // get evaluation of active member
        double V = (*NETypes[i]->pop_member_active)->get_evaluation();
        V = xi_i*(R - V) + V;
        (*NETypes[i]->pop_member_active)->update(V);
    }
    xi = matrix1d(NETypes.size(), 0.0);
}

TypeNeuroEvo::~TypeNeuroEvo(void) {
    for (NeuroEvo* ne : NETypes) {
        delete ne;
    }
}