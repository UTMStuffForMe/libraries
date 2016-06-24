// Copyright 2016 Carrie Rebhuhn
#include "UTMDomainDetail.h"
#include <string>
#include <list>
#include <vector>

// TEMP
#include <assert.h>

using std::vector;
using easymath::XY;
using easymath::zeros;
using std::list;

UTMDomainDetail::UTMDomainDetail(UTMModes* params_set) :
    UTMDomainAbstract(params_set){
    //fix_locs(FileIn::read_pairs<XY>("agent_map/fixes.csv")) {
    // Add internal link IDs to the end of existing linkIDs
    // (important for internal travel)
    /*int cur_edge_num = linkIDs->size();
    for (int i = 0; i < sectors.size(); i++) {
        linkIDs->insert(make_pair(edge(i, i),cur_edge_num + i));
    }*/


    matrix2d membership_map =
        FileIn::read2<double>("agent_map/membership_map.csv");

	// I don't know which order X and Y go in, so this is a guess
	//map_dims = std::make_pair(membership_map.size(), membership_map[0].size());

    // Planning
    lowGraph = new SectorGraphManager(membership_map, highGraph->getEdges());

    // Get link IDs for fix generation
    // TODO: generate a fix somewhere in the airspace -- aim for the center, if not available pick random
    //for (size_t i = 0; i < fix_locs.size(); i++) {
    //    fixes.push_back(new FixDetail(fix_locs[i], i, highGraph, lowGraph,
    //        //&fixes, 
    //        fix_locs,
    //        params, linkIDs));
    //}
    vector<XY> sector_locs(sectors.size());
    for (int i = 0; i < sectors.size(); i++) {
        sector_locs[i] = sectors[i]->xy;
    }
    for (Sector* s : sectors) {
		delete s->generation_pt; // Delete the Fix created by the abstract constructor
        s->generation_pt = new FixDetail(s->xy, s->ID, highGraph, lowGraph, sector_locs, params, linkIDs);
    }

	// This is assuming there is one fix. If there's more and we need to keep track
	// of "done" UAVs, well... Guess we'll have to come up with something else
	for (int s = 0; s < params->n_sectors; s++)
		sectors[s]->generation_pt->UAVs_stationed = &UAVs_done[s];

    // NOTE: MAKE A 'SECTORDETAIL'?
}

UTMDomainDetail::~UTMDomainDetail(void) {
}


void UTMDomainDetail::logUAVLocations() {
    matrix1d stepLocation;
	int i = 0;
	std::list<UAV*> all_UAVs = UAVs;
	for (int s = 0; s < params->n_sectors; s++) {
		for (UAV* ud : UAVs_done[s]) {
			// the UAV
			std::list<UAV*>::iterator successor = std::find_if(all_UAVs.begin(), all_UAVs.end(), [ud](UAV* u) {
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

    for (UAV* u : all_UAVs) {
        stepLocation.push_back(static_cast<UAVDetail*>(u)->loc.x);
        stepLocation.push_back(static_cast<UAVDetail*>(u)->loc.y);
    }
    UAVLocations.push_back(stepLocation);
}


void UTMDomainDetail::exportUAVLocations(int fileID) {
    FileOut::print_vector(UAVLocations,
        "Locations" + std::to_string(fileID) + ".csv");
}

vector<double> UTMDomainDetail::getPerformance() {
    return zeros(1);
    // return matrix1d(sectors.size(),-conflict_count);
}


vector<double> UTMDomainDetail::getRewards() {
    // MAY WANT TO ADD SWITCH HERE

    // DELAY REWARD
    // return zeros(1);
	// links include the internal links, which aren't controlled by agents
    return matrix1d(params->n_links, 0);

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
    return lowGraph->getMembership(p);
}

// HACK: ONLY GET PATH PLANS OF UAVS just generated
void UTMDomainDetail::getPathPlans() {
    // REPLACE WITH PLANPATH
    for (UAV* u : UAVs) {
		u->planAbstractPath(); // plan the abstract path to see if path has changed

		if (static_cast<UAVDetail*>(u)->committed_to_link) {
			bool hasPlan = static_cast<UAVDetail*>(u)->hasDetailPlan();
			bool newSector = u->next_sector_ID == u->cur_sector_ID;
			bool internalLink = u->cur_link_ID < params->n_links && u->next_link_ID >= params->n_links;
			
			if (!hasPlan)
				static_cast<UAVDetail*>(u)->planDetailPath();

			if (newSector && !internalLink) {
				// Kinda hacky. I'm not sure where else I'm going wrong, so I had to do it. :(
				u->mem = u->cur_sector_ID;
				static_cast<UAVDetail*>(u)->planDetailPath();
			}
		}
		else if (u->pathChanged) {
        // sets own next waypoint
	        static_cast<UAVDetail*>(u)->planDetailPath();
		}
		else { /* do nothing */ }
    }
}

void UTMDomainDetail::getPathPlans(const std::list<UAV* > &new_UAVs) {
    for (UAV* u : new_UAVs) {
		if (!static_cast<UAVDetail*>(u)->committed_to_link) // Indicates UAV has reached new sector and 
			static_cast<UAVDetail*>(u)->planDetailPath();  // sets own next waypoint
    }
}


void UTMDomainDetail::incrementUAVPath() {
	for (size_t i = 0; i < links.size(); i++) {
		numUAVsOnLinks[i] = links[i]->traffic[0].size();
	}
	
	vector<UAV*> eligible;              // UAVs eligible to move to next link
	size_t num_agents = links.size() - sectors.size();
	vector<Link*> L = links;
	copy_if(UAVs.begin(), UAVs.end(), back_inserter(eligible), [num_agents, L](UAV* u) {
		int csID = u->curSectorID();
		
		// Is UAV in next sector?
		if (u->next_sector_ID == csID) { // UAV has reached boundary of the next sector
			// Is it the goal sector?
			if (csID == u->mem_end) {
				assert(static_cast<UAVDetail*>(u)->committed_to_link);
				// Is it on internal link?
				if (u->cur_link_ID < num_agents && u->next_link_ID >= num_agents) {
					/* do nothing */
				}
				else {
					L[u->next_link_ID]->move_from(u, L[u->cur_link_ID]);    // At goal sector. Move UAV to internal link.
				}
				return false;
			}
			else {
				// UAV has reached the next non-goal sector. It may have to wait before it can move,
				//	so it is free to choose a different link if necessary
				static_cast<UAVDetail*>(u)->committed_to_link = false;
				return true;            // At end of non-destination link
			}
		}
		else {
			// TYPE IMPLEMENTATION
			// Not yet has it reached next sector
			// move toward next waypoint (next in low-level plan)			
			//static_cast<UAVDetail*>(u)->moveTowardNextWaypoint();
			if (!static_cast<UAVDetail*>(u)->committed_to_link) {
				assert(u->cur_link_ID != u->next_link_ID);
				// UAV still needs to traverse current link
				if (L[u->cur_link_ID]->source == u->curSectorID())
					return false;
				else
					return true; // UAV got to a new sector, but is waiting to traverse next link
			}
			else
				return false; // UAV has already committed to a link and is therefore not waiting
		}
	});

	if (eligible.empty()) {
		/* do nothing; no waiting UAVs */
	}
	else {
		// preemtively commit all eligible UAVs to link
		// those that don't move will uncommit (how rude)
		//for (UAV * u : eligible)
		//	static_cast<UAVDetail*>(u)->committed_to_link = true;
		// This moves all UAVs that are eligible and not blocked
		try_to_move(&eligible);
		// Only those that cannot move are left in eligible
		// std::printf("%i UAVs delayed. \n",eligible.size());

		for (UAV* u : eligible) {
			static_cast<UAVDetail*>(u)->committed_to_link = false;	// UAV is still waiting to move
																	// it can decide to take a different link before it moves
			// adds delay for each eligible UAV not able to move
			agents->add_delay(u);

			// Add 1 to the sector that the UAV is trying to move from
			// Carrie! Would this be correct, do ya think?
			// Different from AbstractDomain
			int n = u->curSectorID(); 
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

	for (UAV* u : UAVs)
		static_cast<UAVDetail*>(u)->moveTowardNextWaypoint();
}

void UTMDomainDetail::try_to_move(vector<UAV*> * eligible_to_move) {
	random_shuffle(eligible_to_move->begin(), eligible_to_move->end());

	size_t el_size;
	do {
		el_size = eligible_to_move->size();
		
		vector<Link*> L = links;
		eligible_to_move->erase(
			remove_if(eligible_to_move->begin(), eligible_to_move->end(),
				[L](UAV* u) {				
			int n = u->next_link_ID; //u->pathChanged ? u->curLinkID() : u->nextLinkID();
			int c = u->cur_link_ID;
			int t = u->type_ID;
			if (!L[n]->at_capacity(t)) {
				assert(L[n]->source == u->cur_sector_ID);
				//assert(n < 40); // Can't be internal link
				L[n]->move_from(u, L[c]);
				static_cast<UAVDetail*>(u)->committed_to_link = true;
				static_cast<UAVDetail*>(u)->planDetailPath();
				return true;
			}
			else {
				return false;
			} }),
			eligible_to_move->end());
	} while (el_size != eligible_to_move->size());
}

void UTMDomainDetail::absorbUAVTraffic() {
	// Deletes UAVs
	vector<Link*> l = links;
	vector<Sector*> S = sectors;
	bool keep = params->_disposal_mode == UTMModes::DisposalMode::KEEP ? true : false;
	if (keep) { // remove UAVs from the domain, but keep track of where they were for later
		// THIS IS ASSUMING 1 FIX PER SECTOR
		for (int s = 0; s < params->n_sectors; s++)
		{
			// points to next UAV that has reached it's goal
			std::list<UAV*>::iterator done = UAVs.begin();
			while (true)
			{
				done = find_if(UAVs.begin(), UAVs.end(), [l, S, s](UAV* u) {
					if (u->mem == u->mem_end && u->curSectorID() == s) {
						FixDetail* fix = (FixDetail*)S[u->curSectorID()]->generation_pt;
						if (fix->atDestinationFix(*static_cast<UAVDetail*>(u))) {
							l[u->cur_link_ID]->remove(u);
							static_cast<UAVDetail*>(u)->committed_to_link = false; // uncommit it since it's done anyway
							u->pathChanged = false; // Why not...
							u->times_reached_goal++;
							return true;
						}
						else {
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
	}
	else { // just get rid of UAVs forever if the reach their respective goals
		UAVs.erase(remove_if(UAVs.begin(), UAVs.end(), [l, S](UAV* u) {
			if (u->mem == u->mem_end) {
				FixDetail* fix = (FixDetail*)S[u->curSectorID()]->generation_pt;
				if (fix->atDestinationFix(*static_cast<UAVDetail*>(u))) {
					l[u->cur_link_ID]->remove(u);
					delete u;
					return true;
				}
				else {
					return false;
				}
			}
			return false;
		}), UAVs.end());
	}
}

void UTMDomainDetail::getNewUAVTraffic() {
	// Generates (with some probability) plane traffic for each sector
	for (Sector* s : sectors) {
		list<UAV*> new_UAVs = s->generation_pt->generateTraffic(*step);
		for (UAV* u : new_UAVs) {
			UAVs.push_back(u);
			links.at(u->cur_link_ID)->add(u);
			// ASK Carrie! ABOUT THIS
			if (!links.at(u->cur_link_ID)->at_capacity(u->type_ID)) {
				static_cast<UAVDetail*>(u)->committed_to_link = true;
			}
		}
	}
}

void UTMDomainDetail::reset() {
	// clear UAVs from links
	for (Link* l : links)
		for (int t = 0; t < n_types; t++)
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


void UTMDomainDetail::exportLog(std::string fid, double ) {
    static int calls = 0;
    calls++;
}

void UTMDomainDetail::detectConflicts() {
    for (list<UAV* >::iterator u1 = UAVs.begin(); u1 != UAVs.end(); ++u1) {
        for (list<UAV* >::iterator u2 = std::next(u1); u2 != UAVs.end(); ++u2) {
            XY a = static_cast<UAVDetail*>(*u1)->loc;
            XY b = static_cast<UAVDetail*>(*u2)->loc;
            double d = easymath::euclidean_distance(a, b);

            if (d > params->get_conflict_thresh()) continue;  // No conflict!

            addConflict(*u1, *u2);
        }
    }
}

void UTMDomainDetail::computeBoundaries() {
	//for (int x = 0; x)
	/*
	int neighbor_left(const XY p) { return (p.x - 1 >= 0) ? lowGraph->getMembership(XY(p.x - 1, p.y)) : -1; }
	int neighbor_right(const XY p) { return (p.x + 1 < map_dims.first) ? lowGraph->getMembership(XY(p.x + 1, p.y)) : -1; }
	int neighbor_up(const XY p) { return (p.y - 1 >= 0) ? lowGraph->getMembership(XY(p.x, p.y - 1)) : -1; }
	int neighbor_down(const XY p) { return (p.y + 1 < map_dims.second) ? lowGraph->getMembership(XY(p.x, p.y + 1)) : -1; }

	int (*neighborhood[4])(const XY p) = { neighbor_left, neighbor_right, neighbor_up, neighbor_down };
	for (int x = 0; x < map_dims.first; x++) {
		for (int y = 0; y < map_dims.second; y++) {
			XY p = XY(x, y);
			int source = lowGraph->getMembership(p);
			for (size_t i = 0; i < 4; i++) {
				int target = neighborhood[i](p);
				if (target < 0) continue; // neighbor is not part o 
			}
		}		
	}
	*/
}