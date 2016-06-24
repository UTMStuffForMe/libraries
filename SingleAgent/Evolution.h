// Copyright 2016 Carrie Rebhuhn
#ifndef SINGLEAGENT_EVOLUTION_H_
#define SINGLEAGENT_EVOLUTION_H_

#include <list>
#include "IAgent.h"
#include "STL/easystl.h"

template <class Policy>
class Evolution : public IAgent<Policy> {
 public:
    //! Life cycle
    Evolution() {}
    Evolution(const Evolution &E) {
        population = new Population();
        for (Policy* p : E.population)
            population->push_back(new Policy(*p));
        PopulationMember* pop_member_active
            = new PopulationMember(population->begin());
    }
    ~Evolution() {
        delete pop_member_active;
        easystl::clear(population);
        delete population;
    }

    //! Mutators
    virtual void generate_new_members() = 0;
    virtual void activate_next_member() {
        (*pop_member_active)++;
        set_policy(**pop_member_active);
    }
    virtual void select_survivors() = 0;
    void update(const Reward &rwd) { policy->update(rwd); }
    void set_first_member() {
        *pop_member_active = population->begin();
        policy = **pop_member_active();
    }


    //! Accessor
    bool at_last_member() const {
        return *pop_member_active == population->end();
    }

 private:
     typedef std::list<Policy*> Population;
     typedef typename std::list<Policy*>::iterator PopulationMember;
     Population* population;
     PopulationMember* pop_member_active;
};

#endif  // SINGLEAGENT_EVOLUTION_H_
