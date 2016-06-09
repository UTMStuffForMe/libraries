// Copyright 2016 Carrie Rebhuhn
#include "UTMDomainDetail.h"
#include <string>
#include <list>
#include <vector>

using std::vector;
using easymath::XY;
using easymath::zeros;
using std::list;
using std::map;

UTMDomainDetail::UTMDomainDetail(UTMModes* params_set) :
    UTMDomainAbstract(params_set, true) {

    matrix2d membership_map =
        FileIn::read2<double>("agent_map/membership_map.csv");

    // Planning
    GridGraph* base = new GridGraph(membership_map);
    lowGraph = new MultiGraph<GridGraph>(highGraph->at()->get_n_edges(), base);

    vector<edge> edges = highGraph->at()->get_edges();
    vector<vector<int> > connections(params_set->n_sectors);
    for (edge e : edges)
        connections[e.first].push_back(e.second);
    
    vector<XY> sector_locs = highGraph->at()->get_locations();
    for (int i = 0; i < params->n_sectors; i++)
        sectors.push_back(new SectorDetail(sector_locs[i], i, connections[i],
            sector_locs, highGraph, lowGraph, params, &UAVs_done[i]));
}


void UTMDomainDetail::logUAVLocations() {
    //~B
    matrix1d stepLocation;
    list<UAVDetail*> all_UAVs = UAVs;
    for (int s = 0; s < params->n_sectors; s++) {
        for (UAVDetail *ud : UAVs_done[s]) {
            // the UAV
            std::list<UAVDetail*>::iterator successor = std::find_if(all_UAVs.begin(), all_UAVs.end(), [ud](UAV* u) {
                if (u->ID > ud->ID)
                    return true;
                else
                    return false;
            });

            if (successor == all_UAVs.end())
                all_UAVs.push_back(ud);
            else
                all_UAVs.insert(successor, ud);
        }

    }

    for (UAVDetail* u : all_UAVs) {
        stepLocation.push_back(u->get_location().x);
        stepLocation.push_back(u->get_location().y);
    }
    UAVLocations.push_back(stepLocation);
}


void UTMDomainDetail::exportUAVLocations(int fileID) {
    // ~B
    //FileOut::print_vector(UAVLocations,
        //"Locations" + std::to_string(fileID) + ".csv");
}

vector<double> UTMDomainDetail::getPerformance() {
    // TODO
    return zeros(1);
}


vector<double> UTMDomainDetail::getRewards() {
    //~B
            // links include the internal links, which aren't controlled by agents
    return matrix1d(params->n_links, 0);

    // DELAY REWARD
    ///return zeros(1);
    // return matrix1d(sectors.size(), -conflict_count);

    // LINEAR REWARD
    // return matrix1d(sectors->size(),-conflict_count);  // linear reward


    // QUADRATIC REWARD
    /* int conflict_sum = 0;
    for (int i=0; i<conflict_count_map->size(); i++){
    for (int j=0; j<conflict_count_map->at(i).size(); j++){
    int c = conflict_count_map->at(i)[j];
    conflict_sum += c*c;
    }
    }
    return matrix1d(sectors->size(),-conflict_sum);*/
}

size_t UTMDomainDetail::getSector(easymath::XY p) {
    // tests membership for sector, given a location
    return lowGraph->at()->get_membership(p);
}

// HACK: ONLY GET PATH PLANS OF UAVS just generated
void UTMDomainDetail::getPathPlans() {
    // REPLACE WITH PLANPATH
    //~B
    for (UAVDetail* u : UAVs) {
        // sets own next waypoint
        u->planAbstractPath(); // plan the abstract path to see if path has changed

//        if (u->committed_to_link) {
            bool hasPlan = u->has_detail_plan();
            bool newSector = u->next_sector_ID == u->cur_sector_ID;
            int next_link = linkIDs->at(edge(u->get_cur_sector(), u->get_next_sector()));
            //bool internalLink = u->cur_link_ID < params->n_links && u->next_link_ID >= params->n_links; // weird logic: moved to UAV
            bool internalLink = u->on_internal_link(next_link);

            if (!hasPlan)
                u->planDetailPath();

            if (newSector && !internalLink) {
                // Kinda hacky. I'm not sure where else I'm going wrong, so I had to do it. :(
                    //u->mem = u->cur_sector_ID;
                u->planDetailPath();

            }
  //      }
        //} else if (u->pathChanged) {
            //u->planDetailPath();

//    } else { /* do nothing */ }
}
}

void UTMDomainDetail::getPathPlans(const list<UAVDetail* > &new_UAVs) {
    for (UAVDetail* u : new_UAVs) {
//        if (u->committed_to_link) // Indicates UAV has reached new sector and 
            u->planDetailPath();  // sets own next waypoint
    }
}


void UTMDomainDetail::incrementUAVPath() {
    matrix1d numUAVsOnLinks(links.size());
    for (size_t i = 0; i < links.size(); i++) {
        numUAVsOnLinks[i] = links[i]->count_traffic();
    }

    vector<UAVDetail*> eligible;  // UAVs eligible to move to next link
    
    copy_if(UAVs.begin(), UAVs.end(), back_inserter(eligible), [](UAVDetail* u) {
        return u->at_boundary();  // Copy if about to transition between sectors
    });

    if (!eligible.empty()) {
        try_to_move(&eligible);
        for (UAVDetail* u : eligible) {
            agents->add_delay(u);
            int n = u->get_cur_sector();
            numUAVsAtSector[n]++;

            // counterfactuals
            if (params->_reward_mode == UTMModes::RewardMode::DIFFERENCE_AVG)
                agents->add_average_counterfactual();
            else if (params->_reward_mode ==
                UTMModes::RewardMode::DIFFERENCE_DOWNSTREAM)
                agents->add_downstream_delay_counterfactual(u);
            else
                continue;
        }

    }

    for (UAVDetail* u : UAVs)
        u->moveTowardNextWaypoint();

}

void UTMDomainDetail::reset() {
    // clear UAVs from links
    for (Link* l : links)
        for (size_t t = 0; t < n_types; t++)
            l->traffic[t].clear();

    // delete UAVs
    for (UAV* u : UAVs)
        delete u;
    UAVs.clear();

    for (int s = 0; s < params->n_sectors; s++) {
        for (UAV* ud : UAVs_done[s])
            delete ud;
        UAVs_done[s].clear();

    }
    UAVLocations.clear();
}

void UTMDomainDetail::try_to_move(vector<UAVDetail*> * eligible_to_move) {
    random_shuffle(eligible_to_move->begin(), eligible_to_move->end());


    size_t el_size;
    do {
        el_size = eligible_to_move->size();

        vector<Link*> L = links;
        map<edge, int>* ids = linkIDs;
        eligible_to_move->erase(
            remove_if(eligible_to_move->begin(), eligible_to_move->end(),
                [L, ids](UAVDetail* u) {
            int n = ids->at(edge(u->get_cur_sector(), u->get_next_sector()));
            int c = u->get_cur_link();
            int t = u->get_type();
            if (!L[n]->at_capacity(t)) {
                L[n]->move_from(u, L[c]);
                u->planDetailPath();
                return true;

            } else {
                return false;

            } }),
            eligible_to_move->end());

    } while (el_size != eligible_to_move->size());

}



void UTMDomainDetail::exportLog(std::string fid, double) {
    static int calls = 0;
    calls++;
}

void UTMDomainDetail::detectConflicts() {
    for (list<UAVDetail* >::iterator u1 = UAVs.begin(); u1 != UAVs.end(); ++u1) {
        for (list<UAVDetail* >::iterator u2 = std::next(u1); u2 != UAVs.end(); ++u2) {
            XY a = (*u1)->get_location();
            XY b = (*u2)->get_location();
            double d = easymath::euclidean_distance(a, b);

            if (d > params->get_conflict_thresh()) continue;  // No conflict!

            addConflict(*u1, *u2);
        }
    }
}

void UTMDomainDetail::absorbUAVTraffic() {
    // Deletes UAVs
    vector<Link*> l = links;
    vector<SectorDetail*> S = sectors;
    bool keep = params->_disposal_mode == UTMModes::DisposalMode::KEEP ? true : false;
    if (keep) { // remove UAVs from the domain, but keep track of where they were for later
              // THIS IS ASSUMING 1 FIX PER SECTOR
        for (int s = 0; s < params->n_sectors; s++)
        {
            // points to next UAV that has reached it's goal
            std::list<UAVDetail*>::iterator done = UAVs.begin();
            while (true)
            {
                done = find_if(UAVs.begin(), UAVs.end(), [l, S, s](UAVDetail *u) {
                    if (u->at_destination()) {
                        FixDetail* fix = S[u->get_cur_sector()]->generation_pt; // note: may want to move this to internal to fixes, for now making generation pt public
                        if (fix->atDestinationFix(*u)) {
                            l[u->get_cur_link()]->remove(u);
//                            u->committed_to_link = false; // uncommit it since it's done anyway
//                            u->pathChanged = false; // Why not...
                            u->times_reached_goal++;
                            return true;

                        } else {
                            return false;

                        }

                    }
                    return false;
                });
                if (done == UAVs.end())
                    break;
                else
                    UAVs_done[s].splice(UAVs_done[s].begin(), UAVs, done, std::next(done));
            }
        }

    } else { // just get rid of UAVs forever if they reach their respective goals
        UAVs.erase(remove_if(UAVs.begin(), UAVs.end(), [l, S](UAVDetail* u) {
            if (u->at_destination()) {
                FixDetail* fix = (FixDetail*)S[u->get_cur_sector()]->generation_pt;
                if (fix->atDestinationFix(*u)) {
                    l[u->get_cur_link()]->remove(u);
                    delete u;
                    return true;

                } else {
                    return false;

                }

            }
            return false;
        }), UAVs.end());

    }

}

void UTMDomainDetail::getNewUAVTraffic() {
    for (SectorDetail* s : sectors) {
        UAVDetail* u = s->generation_pt->generate_UAV(*step);
        if (u == NULL) continue;
        
        links.at(u->get_cur_link())->add(u);
        UAVs.push_back(u);
    }
}
