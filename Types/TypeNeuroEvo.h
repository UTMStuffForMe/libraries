// Copyright 2016 Carrie Rebhuhn
#ifndef SINGLEAGENT_NEUROEVO_TYPENEUROEVO_H_
#define SINGLEAGENT_NEUROEVO_TYPENEUROEVO_H_

#include <algorithm>
#include <functional>
#include <vector>

#include "NeuroEvo.h"
#include "Math/easymath.h"
#include "SingleAgent/NeuralNet/TypeNeuralNet.h"

class TypeNeuroEvo : public Evolution<TypeNeuralNet> {
 public:
     //! Life cycle
    TypeNeuroEvo(void) {}
    TypeNeuroEvo(NeuroEvoParameters* NEParams, size_t nTypes);
    void deep_copyNETypes(const std::vector<NeuroEvo*> &NETypesSet);
    ~TypeNeuroEvo(void);

    //! Accessors
    matrix1d getBestMemberValAll();
    matrix1d get_action(matrix1d state);
    matrix1d get_action(matrix1d state, size_t neighbor_type);
    matrix1d get_action(matrix2d state);

    //! Mutators
    void generate_new_members();
    bool select_new_memberAll();
    void select_survivorsAll();
    void update_policy_values(double R);

    //! Members
    std::vector<NeuroEvo*> NETypes; // the set of neuro-evo instances for each type in the system
    matrix1d xi;  // eligibility trace: count of how many times each neural net used in run
};
#endif  // SINGLEAGENT_NEUROEVO_TYPENEUROEVO_H_
