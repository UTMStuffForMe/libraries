// Copyright 2016 Carrie Rebhuhn
#include "UTMDomainAbstract.h"
#include <vector>
#include <string>
#include <list>
#include <map>

using std::list;
using std::vector;
using std::string;
using easymath::XY;
using std::ifstream;
using std::map;
using easymath::zeros;

UTMDomainAbstract::UTMDomainAbstract(UTMModes* params_set, bool only_abstract): 
    IDomainStateful(params_set) {
    // This will only partially initialize UTMDomain
    // This will not initialize any variables that have overrides
    filehandler = new UTMFileNames(params_set),
        params = params_set;

    // Airspace construction
    
    // Directories
    string domain_dir = filehandler->createDomainDirectory(); // NOTE: MODIFY THIS TO LOOP OVER THE DOMAIN FILES...
    string efile = domain_dir + "edges.csv";
    string vfile = domain_dir + "nodes.csv";
    
    // Variables to fill 
    vector<edge> edges;
    vector<XY> locs;
    if (params->_airspace_mode == UTMModes::AirspaceMode::SAVED && FileIn::file_exists(efile)) {
        edges = FileIn::read_pairs<edge>(efile);
        locs = FileIn::read_pairs<XY>(vfile);

        LinkGraph* base = new LinkGraph(locs, edges);
        highGraph = new MultiGraph<LinkGraph>(n_types, base);
    } else {
        // Generate a new airspace
        LinkGraph* base = new LinkGraph(params->get_n_sectors(), 200, 200);
        highGraph = new MultiGraph<LinkGraph>(n_types, base);
        highGraph->at()->print_graph(domain_dir);  // saves the graph

        locs = highGraph->at()->get_locations();
        edges = highGraph->at()->get_edges();
    }

    // n_links must be set after graph created
    params->n_links = edges.size();
    n_agents = params->get_n_agents(); // must be called after n_links populated
    int n_sectors = locs.size();

    // Link construction
    linkIDs = new map<edge, int>();
    vector<vector<int> > connections(n_sectors);
    for (edge e : edges) {
        int source = e.first;   // membership of origin of edge
        int target = e.second;  // membership of connected node
        XY s_loc = locs[source];
        XY t_loc = locs[target];
        int cardinal_dir = cardinal_direction(s_loc - t_loc);
        int dist = static_cast<int>(euclidean_distance(s_loc, t_loc)/10.0);
        if (dist == 0) dist = 1;
        size_t cap = static_cast<size_t>(params->get_flat_capacity());
        links.push_back(
            new Link(links.size(), source, target, dist,
                vector<size_t>(n_types, cap), cardinal_dir));
        linkIDs->insert(std::make_pair(e, links.size() - 1));

        connections[source].push_back(target);
        incoming_links[target].push_back(source);
    }


    if (params->_agent_defn_mode == UTMModes::AgentDefinition::SECTOR)
        agents = new SectorAgentManager(links, n_types, sectors, params);
    else
        agents = new LinkAgentManager(links.size(), n_types, links, params);

    numUAVsAtSector = zeros(n_sectors);
}

UTMDomainAbstract::UTMDomainAbstract(UTMModes* params_set): UTMDomainAbstract(params_set, true) {
    // Sector/Fix  construction
    vector<edge> edges = highGraph->at()->get_edges();
    vector<vector<int> > connections(params_set->n_sectors);
    for (edge e : edges)
        connections[e.first].push_back(e.second);

    vector<XY> sector_locs = highGraph->at()->get_locations();
    for (int i = 0; i < params->n_sectors; i++) {
        Sector* s = new Sector(sector_locs[i], i, connections[i], sector_locs, highGraph, params);
        s->generation_pt = new Fix(s->xy, s->ID, highGraph, sector_locs, params);
        sectors.push_back(s);
    }
}

string UTMDomainAbstract::createExperimentDirectory() {
    return filehandler->createExperimentDirectory();
}

UTMDomainAbstract::~UTMDomainAbstract(void) {
    delete linkIDs;
    delete filehandler;
    delete agents;

    for (Link* l : links)
        delete l;

    for (Sector* s : sectors)
        delete s;

    for (UAV* u : UAVs)
        delete u;

    for (int s = 0; s < params->n_sectors; s++)
        for (UAV* ud : UAVs_done[s])
            delete ud;
}

matrix1d UTMDomainAbstract::getPerformance() {
    return agents->performance();
}

matrix1d UTMDomainAbstract::getRewards() {
    return agents->reward();
}


void UTMDomainAbstract::incrementUAVPath() {
    vector<UAV*> eligible;              // UAVs eligible to move to next link
    copy_if(UAVs.begin(), UAVs.end(), back_inserter(eligible), [](UAV* u) {
        if (u->at_link_end()) {
            if (u->at_terminal_link()) {
                return false;
            } else {
            // TYPE IMPLEMENTATION
            for (size_t i = 0; i <= u->get_type(); i++)
                u->decrement_wait();
            return false;
        }
    });

    if (eligible.empty()) {
        return;
    } else {
        // This moves all UAVs that are eligible and not blocked
        try_to_move(&eligible);
        // Only those that cannot move are left in eligible
        // std::printf("%i UAVs delayed. \n",eligible.size());

        for (UAV* u : eligible) {
            // adds delay for each eligible UAV not able to move
            agents->add_delay(u);

            // Add 1 to the sector that the UAV is trying to move from
            numUAVsAtSector[u->get_next_sector()]++;

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
}

void UTMDomainAbstract::try_to_move(vector<UAV*> * eligible_to_move) {
    random_shuffle(eligible_to_move->begin(), eligible_to_move->end());

    size_t el_size;
    do {
        el_size = eligible_to_move->size();

        vector<Link*> L = links;
        
        map<edge, int>* L_IDs = linkIDs;
        eligible_to_move->erase(
            remove_if(eligible_to_move->begin(), eligible_to_move->end(),
                [L,L_IDs](UAV* u) {

            //printf("Got here");
            if (u->at_terminal_link()) {
                return false;
            }

            // Get next link ID
            size_t n = L_IDs->at(edge(u->get_nth_sector(1), u->get_nth_sector(2)));
            // Get current link ID
            size_t c = u->get_cur_link();
            size_t t = u->get_type();

            //printf("\n...%i,%i,%i... ", n, c, t);

            //printf("UAV %i wants to hop from %i  to link %i. Link % has ",u->ID,c, n,n);
            //for (list<UAV*> tout : L[n]->traffic) {
                //for (UAV* t : tout) {
                //    printf("%i: ", t->ID);
              //  }

            //}

            if (!L[n]->at_capacity(t)) {
                L[n]->move_from(u, L[c]);
                //printf("--success!\n");
                //system("pause");
                return true;
            } else {
                return false;
            } }),
            eligible_to_move->end());
    } while (el_size != eligible_to_move->size());
}

matrix2d UTMDomainAbstract::getStates() {
    matrix2d allStates(n_agents, matrix1d(n_state_elements, 0.0));

    /* "NORMAL POLARITY" state
    for (UAV* u : UAVs){
        allStates[getSector(u->loc)][u->get_travel_direction()]+=1.0; // Adds the UAV impact on the state
    }
    */

    // REVERSED POLARITY STATE
    /*for (UAV* u : UAVs){
        // COUNT THE UAV ONLY IF IT HAS A NEXT SECTOR IT'S GOING TO
        if (u->get_next_sector()==u->get_cur_sector()) continue;
        else allStates[u->get_next_sector()][u->get_travel_direction()] += 1.0;
    }*/

    // CONGESTION STATE

    if (params->_agent_defn_mode == UTMModes::AgentDefinition::SECTOR) {
        vector<int> sector_congestion_count(n_agents, 0);
        for (UAV* u : UAVs) {
            sector_congestion_count[u->get_cur_sector()]++;
        }
        for (size_t i = 0; i < sectors.size(); i++) {
            for (int conn : sectors[i]->connections) {
                XY dx = sectors[i]->xy - sectors[conn]->xy;
                size_t dir = cardinal_direction(dx);
                allStates[i][dir]
                    += sector_congestion_count[conn];
            }
        }
    } else {
        for (UAV* u : UAVs)
            allStates[u->get_cur_link()][u->get_type()]++;
    }


    agents->agentStates.push_back(allStates);

    return allStates;
}


void UTMDomainAbstract::simulateStep(matrix2d agent_actions, int) {
    // Alter the cost maps (agent actions)
    agents->logAgentActions(agent_actions);
    bool action_changed = agents->last_action_different();

    // New UAVs appear
    this->getNewUAVTraffic();

    if (action_changed) {
        matrix2d w = agents->actions2weights(agent_actions);
        for (size_t i = 0; i < n_types; i++) {
            highGraph->at(i)->set_weights(w[i]);
        }
    }

    // Make UAVs reach their destination
    absorbUAVTraffic();

    // Plan over new cost maps
    if (action_changed)
        getPathPlans();

    // UAVs move
    incrementUAVPath();
    if (params->_reward_type_mode == UTMModes::RewardType::CONFLICTS)
        detectConflicts();

    // At end of the step
    //printf("UAVS:%i\n", UAVs.size());
}

// Records information about a single step in the domain
void UTMDomainAbstract::logStep() {
    if (params->_agent_defn_mode == UTMModes::AgentDefinition::SECTOR
        || params->_agent_defn_mode == UTMModes::AgentDefinition::LINK) {
        matrix1d numUAVsOnLinks(links.size(), 0);
        for (size_t i = 0; i < links.size(); i++) {
            numUAVsOnLinks[i] = links[i]->traffic[0].size();
        }
        linkUAVs.push_back(numUAVsOnLinks);

        sectorUAVs.push_back(numUAVsAtSector);
        numUAVsAtSector = zeros(sectors.size());
    }
}

// This function is called in SimNE.cpp. The first argument is the number
// of the NN team (out of 20 or w/e pop size is) with the best performance.
// Second arg is used to differentiate between different points in evolution
// (trained or untrained, for example).
// This function is called once per full simulation (words?), after the last
// epoch
// TODO(Brandon) -- we should probably dump this data to a file before
// moving to the next neural evaluation
void UTMDomainAbstract::exportStepsOfTeam(int team, std::string suffix) {

    if (linkUAVs.size() == 0) return;
    int start = params->get_n_steps()*team;
        // 200 * team;
    // for (int i = 0; i < 200; i++) {

    matrix2d link_log, sector_log;
    for (int i = 0; i < params->get_n_steps(); i++) {
        link_log.push_back(linkUAVs[start + i]);
        sector_log.push_back(sectorUAVs[start + i]);
    }

    // Save history of traffic in csv files
    // Maybe it's good to change where these are saved
    FileOut::print_vector(link_log, "link_UAVs_" + suffix + ".csv");
    FileOut::print_vector(sector_log, "sector_UAVs_" + suffix + ".csv");
    linkUAVs.clear();
    sectorUAVs.clear();
}

matrix3d UTMDomainAbstract::getTypeStates() {
    matrix3d allStates = zeros(n_agents, n_types, n_state_elements);

    matrix2d state_printout = zeros(n_agents, n_state_elements);
    if (params->_agent_defn_mode == UTMModes::AgentDefinition::SECTOR) {
        for (UAV* u : UAVs) {
            int a = u->get_cur_sector();
            size_t id = u->get_type();
            int dir = u->get_travel_direction();
            allStates[a][id][dir] += 1.0;
            state_printout[a][dir]++;
        }
    } else {
        // link definition
        for (UAV* u : UAVs) {
            size_t a = u->get_cur_link();
            size_t id = u->get_type();
            allStates[a][id][0] += 1.0;
        }
            //~B
           /* if (a >= params->n_links) // internal link?
            {
                // When counting UAVs in internal links,
                    // we'll add a "portion" of the UAV to the
                    // nearby links --- we should talk about the rationale for this? - Carrie
                int target = a - params->n_links;
                std::list<int> incoming = incoming_links[target];
                for (std::list<int>::iterator it = incoming.begin(); it != incoming.end(); it++)
                {
                    a = *it;
                    allStates[a][id][0] += 1.0 / (double)incoming.size();
    }
            } else
            {
                allStates[a][id][0] += 1.0;
            }
        }*/
    }
    agents->agentStates.push_back(state_printout);
    return allStates;
}

void UTMDomainAbstract::exportSectorLocations(int fileID) {
    std::vector<easymath::XY> sectorLocations;
    for (Sector* s : sectors)
        sectorLocations.push_back(s->xy);
    FileOut::print_pair_container(sectorLocations,
        "visualization/agent_locations" + std::to_string(fileID) + ".csv");
}

void UTMDomainAbstract::detectConflicts() {
    agents->detect_conflicts();
    // CURRENTLY CONFLICT DISABLED


    // if (params->_agent_defn_mode==UTMModes::SECTOR){
    // matrix1d G_c(sectors.size());
    // }

    // count the over capacity here

    // Calculate the amount OVER or UNDER the given capacity
    // COUNT UP SECTOR CAPACITIES

    // matrix2d cap = sectorCapacity;

    // Global reward SECTORS
    /*for (UAV* u: UAVs)
        if ((cap[u->get_cur_sector()][u->get_type()]--)<0)
            conflict_count++;

    // D average SECTORS
    cap = sectorCapacity;
    for (UAV* u: UAVs)
        if ((cap[u->get_cur_sector()][u->get_type()]--)<0)
            for (int j=0; j<n_types; j++)
                sectors->at(u->get_cur_sector()).conflicts[j]++; // D avg


    // D downstream SECTORS
    cap = sectorCapacity;
    for (UAV* u: UAVs)
        if ((cap[u->get_cur_sector()][u->get_type()]--)<0)
            for (uint i=0; i<sectors->size(); i++)
                if (u->sectors_touched.find(i)==u->sectors_touched.end())
                    conflict_minus_downstream[i]++;	/// D downstream

    // D reallocation SECTORS
    for (uint i=0; i<sectors->size(); i++){
        matrix2d cap_i = cap;
        matrix1d occ_i = cap[i];
        cap_i[i] = matrix1d(cap_i[0].size(),0);

        for (int j=0; j<n_types; j++){
            while (occ_i[j]<0){ // ONLY TAKE OUT THE OVER CAPACITY--ASSUME PERFECT ROUTING?
                // add back up to capacity
                occ_i[j]++;

                int alt;
                do {
                    alt = rand()%n_agents;
                } while(alt==i);
                cap_i[alt][j]--;
            }
        }
        for (uint j=0; j<cap_i.size(); j++)
            for (uint k=0; k<cap_i[j].size(); k++)
                if (cap_i[j][k]<0)
                    conflict_random_reallocation[i]++;
    }

    // Global reward LINKS
    for (Link* l: links)
        conflict_count += l->get_conflicts();

    // D average LINKS
/*	cap = *linkCapacity;
    for (UAV* u: UAVs)
        if ((cap[u->get_cur_link()][u->get_type()]--)<0)
            for (int j=0; j<n_types; j++)
                linkConflicts[u->get_cur_link()][j]++; // D avg
    for (int i=0; i<n_links; i++)
        linkSteps[i]++; // steps of conflict accounting (for average counterfactual)


    // D downstream SECTORS
    cap = *linkCapacity;
    for (UAV* u: UAVs)
        if ((cap[u->get_cur_link()][u->type_ID]--)<0)
            for (int i=0; i<n_links; i++)
                if (u->links_touched.find(i)==u->links_touched.end())
                    link_conflict_minus_downstream[i]++;	/// D downstream
                    */
                    // D reallocation SECTORS
                    /*
                    for (int i=0; i<n_links; i++){
                        matrix2d cap_i = cap;
                        matrix1d occ_i = cap[i];
                        cap_i[i] = matrix1d(cap_i[0].size(),0);

                        for (int j=0; j<n_types; j++){
                            while (occ_i[j]<0){ // ONLY TAKE OUT THE OVER CAPACITY--ASSUME PERFECT ROUTING?
                                // add back up to capacity
                                occ_i[j]++;

                                int alt;
                                do {
                                    alt = rand()%n_links;
                                } while(alt==i);
                                cap_i[alt][j]--;
                            }
                        }
                        for (uint j=0; j<cap_i.size(); j++)
                            for (uint k=0; k<cap_i[j].size(); k++)
                                if (cap_i[j][k]<0)
                                    link_conflict_(om_reallocation[i]++;
                    }
                    */
}

void UTMDomainAbstract::getPathPlans() {
    for (UAV* u : UAVs) {
        if (u->at_link_end())
            u->planAbstractPath();
    }
}

void UTMDomainAbstract::getPathPlans(const std::list<UAV* > &new_UAVs) {
    for (UAV* u : new_UAVs) {
        u->planAbstractPath();  // sets own next waypoint
    }
}

void UTMDomainAbstract::reset() {
    while (!UAVs.empty()) {
        delete UAVs.back();
        UAVs.pop_back();
    }

    for (Link* l : links) {
        l->reset();
    }

    agents->reset();
}

void UTMDomainAbstract::absorbUAVTraffic() {
    // Deletes UAVs
    vector<Link*> l = links;
    UAVs.erase(remove_if(UAVs.begin(), UAVs.end(), [l](UAV* u) {
        if (u->at_link_end() &&  u->at_terminal_link()) {
            l[u->get_cur_link()]->remove(u);
            delete u;
            return true;
        } else {
            return false;
        }
    }), UAVs.end());
}


void UTMDomainAbstract::getNewUAVTraffic() {
    // Generates (with some probability) plane traffic for each sector
    for (Sector* s : sectors) {
        UAV* u = s->generation_pt->generate_UAV(*step);
        if (u == NULL) continue;

        // UAV initially does not have its current link set -- must define from sector pair
        edge u_start_edge = edge(u->get_cur_sector(), u->get_next_sector());
        u->set_cur_link_ID(linkIDs->at(u_start_edge));
            links.at(u->get_cur_link())->add(u);
        UAVs.push_back(u);
    }
}
