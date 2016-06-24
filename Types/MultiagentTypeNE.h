// Copyright 2016 Carrie Rebhuhn
#ifndef MULTIAGENT_MULTIAGENTTYPENE_H_
#define MULTIAGENT_MULTIAGENTTYPENE_H_

#include <vector>
#include <string>
#include "MultiagentNE.h"
#include "SingleAgent/NeuroEvo/TypeNeuroEvo.h"
#include "SingleAgent/NeuroEvo/NeuroEvo.h"
#include "SingleAgent/NeuroEvo/NeuroEvoTypeWeighted.h"
#include "SingleAgent/NeuroEvo/NeuroEvoTypeCrossweighted.h"


// Container for collection of 'Type Neuro Evo' agents
class MultiagentTypeNE : public IMultiagentSystem<TypeNeuroEvo> {
 public:
    // options for handling different types
    enum TypeHandling { BLIND, WEIGHTED, CROSSWEIGHTED, MULTIMIND, NMODES };
    TypeHandling type_mode;
    int n_types;
    
    std::vector<TypeNeuroEvo*> agents;

    // Life cycle
    MultiagentTypeNE(void) {}
    MultiagentTypeNE(int n_agents, NeuroEvoParameters* NE_params,
        TypeHandling type_mode, int n_types);
    ~MultiagentTypeNE(void) {};

    // void initializeWithStereotypes(std::vector<std::vector<NeuroEvo*> >
    // stereotypes, std::vector<int> agent_types);
   // std::vector<Action> get_actions(std::vector<std::vector<State > > state);

    //! Returns true if multiple neural nets are used
    //! Currently only done in the multimind case
    //bool multiple_nets() {
      //  return type_mode == MULTIMIND;
    //}

    //! Select the next member and return true if not at the end
    //! Specific to Evo
    virtual bool setNextPopMembers() {
        std::vector<bool> not_end(agents.size(), false);
        for (size_t i = 0; i < agents.size(); i++) {
            //if (multiple_nets()) {
                //not_end[i] = static_cast<TypeNeuroEvo*>
                    //(agents[i])->select_new_memberAll();
            agents[i]->select_new_memberAll();
            //} else {
              //  not_end[i] = static_cast<NeuroEvo*>
                //    (agents[i])->select_new_member();
            //}
        }
        for (bool a : not_end) {
            if (!a) {
                return false;
            }
        }

        return true;
    }
    std::string type_file_name() {
        std::string typefilenames[MultiagentTypeNE::TypeHandling::NMODES] = {
        "blind",
        "weighted",
        "crossweighted",
        "multimind",
        };
        return typefilenames[type_mode];
    }

    virtual void select_survivors() {
        // Specific to Evo: select survivors
        for (TypeNeuroEvo* a : agents) {
            //if (type_mode == MULTIMIND) {
                a->select_survivorsAll();
            //} else if (type_mode == WEIGHTED ||
              //  type_mode == CROSSWEIGHTED ||
                //type_mode == BLIND) {
                //static_cast<NeuroEvo*>(a)->select_survivors();
            }
        }
    //}
};
#endif  // MULTIAGENT_MULTIAGENTTYPENE_H_
