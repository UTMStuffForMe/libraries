// Copyright 2016 Carrie Rebhuhn
#include "Link.h"

#include <algorithm>
#include <functional>
#include <list>
#include <vector>

Link::Link(int ID, int source_set, int target_set,
    int time, std::vector<size_t> capacity, int cardinal_dir) :
    ID(ID),
    source(source_set),
    target(target_set),
    time(time),
    cardinal_dir(cardinal_dir),
    capacity(capacity),
    traffic(size_t(UTMModes::UAVType::NTYPES), std::list<UAV*>())
{}

bool Link::at_capacity(size_t UAV_type) {
    return number_over_capacity(UAV_type) >= 0;
}

int Link::number_over_capacity(size_t type_ID) {
    return static_cast<int>(traffic[type_ID].size() - capacity[type_ID]);
}

matrix1d Link::predicted_traversal_time() {
    // Get predicted wait time for each type of UAV
    matrix1d predicted(traffic.size());
    for (size_t i = 0; i < traffic.size(); i++) {
        // Collect wait times on all UAVs ON the link
        matrix1d waits;
        for (UAV* u : traffic[i]) {
            waits.push_back(u->t);
        }

        // Sort by wait (descending)
        std::sort(waits.begin(), waits.end(), std::greater<double>());

        size_t n_ok = capacity[i] - 1;  // UAVs you don't have to wait for
        size_t n_wait = waits.size() - n_ok;  // UAVs before you in line
        if (waits.size() > n_ok)
            waits.resize(n_wait);


        // Store predicted link time.
        double w = easymath::sum(waits);
        predicted[i] = time + w;
    }
    return predicted;
}

void Link::move_from(UAV* u, Link* l) {
    // Add to other list (u is temporarily duplicated)
    add(u);

    // Remove from previous node (l)
    l->remove(u);
}

void Link::add(UAV* u) {
    u->t = time;
    traffic.at(size_t(u->type_ID)).push_back(u);
    u->cur_link_ID = ID;
    u->mem = source;
}


void Link::remove(UAV* u) {
    traffic[size_t(u->type_ID)].erase(
        std::find(traffic[size_t(u->type_ID)].begin(),
            traffic[size_t(u->type_ID)].end(), u));
}

void Link::reset() {
    traffic = std::vector < std::list<UAV*> >
        (size_t(UTMModes::UAVType::NTYPES), std::list<UAV*>());
}

LinkAgentManager::LinkAgentManager(int n_edges, int n_types,
    std::vector<Link*> links, UTMModes* params) :
    n_edges(n_edges), n_types(n_types), IAgentManager(params), links(links)
{};

matrix2d LinkAgentManager::actions2weights(matrix2d agent_actions) {
    matrix2d weights = easymath::zeros(n_types, n_edges);
    double alpha = 1000.0;

    for (int i = 0; i < n_edges; i++) {
        matrix1d predicted = links.at(i)->predicted_traversal_time();
        for (int t = 0; t < n_types; t++) {
            // note: is there a scalable metric here?
            weights[t][i] = predicted[t] + agent_actions[i][t] * alpha;
            // weights[t][i] = agent_actions[i][t]*1000.0;
        }
    }
    return weights;
}

void LinkAgentManager::add_delay(UAV* u) {
    metrics.at(u->cur_link_ID).local[size_t(u->type_ID)]++;
}

void LinkAgentManager::add_downstream_delay_counterfactual(UAV* u) {
    // remove the effects of the UAV for the counterfactual..
    // calculate the G that means that the UAV's impact is removed...

    if (square_reward) {
        // Non-functional, todo
        printf("SQUARED TODO");
        exit(1);
    } else {
        for (size_t i = 0; i < metrics.size(); i++) {
            if (u->links_touched.count(i) == 0) {
                metrics[i].G_minus_downstream[size_t(u->type_ID)]++;
            } else {
                continue;
            }
        }
    }
}

void LinkAgentManager::detect_conflicts() {
    for (size_t i = 0; i < links.size(); i++) {
        for (size_t j = 0; j < links[i]->traffic.size(); j++) {
            int over_capacity = links[i]->number_over_capacity(j);
            if (over_capacity <= 0)
                continue;  // no congestion
            else if (square_reward)
                metrics[i].local[j] += over_capacity*over_capacity;
            else
                metrics[i].local[j] += over_capacity;
        }
    }
}
