// Copyright 2016 Carrie Rebhuhn
#ifndef SINGLEAGENT_NEUROEVO_NEUROEVOTYPEWEIGHTED_H_
#define SINGLEAGENT_NEUROEVO_NEUROEVOTYPEWEIGHTED_H_

#include <list>
#include "NeuroEvo.h"
#include "SingleAgent/NeuralNet/TypeNeuralNet.h"

class NeuroEvoTypeWeighted : public NeuroEvo {
 public:
    NeuroEvoTypeWeighted(NeuroEvoParameters* NE_params, int n_types,
        int n_state_elements) :
        NeuroEvo(NE_params), n_types(n_types),
        n_state_elements(n_state_elements) {
        matrix3d preprocess_weights
            = easymath::zeros(n_types, n_state_elements, 1);
        deletePopulation();


        // Neural network parameters
        // NOTE: this is after the "preprocess" step
        int INPUT = n_state_elements;
        int OUTPUT = params->nOutput;
        int HIDDEN = params->nHidden;
        for (int i = 0; i < NE_params->popSize; i++) {
            population.push_back(
                new TypeNeuralNet(INPUT, HIDDEN, OUTPUT, preprocess_weights));
        }
        params->nInput = INPUT;

        pop_member_active = population.begin();
    }

    virtual void generate_new_members() {
        // Mutate existing members to generate more
        std::list<NeuralNet*>::iterator popMember = population.begin();
        // add k new members
        for (int i = 0; i < params->popSize; i++) {
            // commented out so that you take parent's evaluation
            // (*popMember)->evaluation = 0.0;
            // dereference pointer AND iterator
            TypeNeuralNet* m
                = new TypeNeuralNet(*static_cast<TypeNeuralNet*>
                    (*popMember));
            m->mutate();
            population.push_back(m);
            ++popMember;
        }
    }

    size_t n_types, n_state_elements;

    using NeuroEvo::get_action;  // so that the overloaded base class is seen

    matrix1d get_action(matrix2d state) {
        // state has elements [type][state element]
        matrix1d preprocessed_state(n_state_elements, 0.0);
        for (size_t s = 0; s < n_state_elements; s++) {
            for (size_t t = 0; t < n_types; t++) {
                preprocessed_state[s] += state[t][s]
                    * (static_cast<TypeNeuralNet*>
                        (*pop_member_active))->preprocess_weights[t][s][0];
            }
        }
        return get_action(preprocessed_state);
    }

    ~NeuroEvoTypeWeighted() {}
};
#endif  // SINGLEAGENT_NEUROEVO_NEUROEVOTYPEWEIGHTED_H_
