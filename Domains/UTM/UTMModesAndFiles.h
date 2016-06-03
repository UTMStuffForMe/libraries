// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_UTMMODESANDFILES_H_
#define DOMAINS_UTM_UTMMODESANDFILES_H_

#include <string>
#include <boost/filesystem/operations.hpp>

#include "Domains/IDomainStateful.h"

class UTMModes : public IDomainStatefulParameters {
public:
    UTMModes() :
        // Mode defaults
        //_reward_mode(UTMModes::RewardMode::GLOBAL),
        _reward_mode(UTMModes::RewardMode::DIFFERENCE_AVG),
        _airspace_mode(UTMModes::AirspaceMode::SAVED),
        _traffic_mode(UTMModes::TrafficMode::DETERMINISTIC),
        _agent_defn_mode(UTMModes::AgentDefinition::LINK),
        _reward_type_mode(UTMModes::RewardType::DELAY),
        _search_type_mode(UTMModes::SearchDefinition::ASTAR),
        // Constants defaults
        square_reward(false),
        n_sectors(5),
        alpha(1000.0),
        domain_num(-1)
    {};
    ~UTMModes() {}

    int domain_num;
    double alpha; // amount that a neural network impacts the system

    // OPTION HERE FOR ONE AGENT PER LINK
    enum class AgentDefinition { SECTOR, LINK };
    AgentDefinition _agent_defn_mode;

    enum class SearchDefinition { ASTAR, RAGS };
    SearchDefinition _search_type_mode;


    // NUMBER OF SECTORS
    int n_sectors;
    int get_n_sectors() {
        return n_sectors;
    }

    // Agents
    int get_n_agents() {
        try {
            if (_agent_defn_mode == UTMModes::AgentDefinition::SECTOR) {
                return get_n_sectors();
            } else if (_agent_defn_mode == UTMModes::AgentDefinition::LINK) {
                return get_n_links();
            } else {
                throw std::runtime_error("Bad _agent_defn_mode");
            }
        }
        catch (std::runtime_error) {
            printf("Bad agent defn!");
            exit(1);
        }
    }

    // This should be set after the graph is constructed!
    int n_links;
    int get_n_links() {
        return n_links;
    }

    // REWARDS
    enum class RewardMode {
        GLOBAL,
        DIFFERENCE_DOWNSTREAM,
        DIFFERENCE_TOUCHED,
        DIFFERENCE_REALLOC,
        DIFFERENCE_AVG,
        NMODES
    };
    bool square_reward;

    RewardMode _reward_mode;
    std::string getRewardModeName() {
        std::string reward_names[size_t(RewardMode::NMODES)] = {
            "GLOBAL",
            "DIFFERENCE_DOWNSTREAM",
            "DIFFERENCE_TOUCHED",
            "DIFFERENCE_REALLOC",
            "DIFFERENCE_AVG"
        };
        return reward_names[size_t(_reward_mode)];
    }

    // This is which types of environment variable is counted
    enum class RewardType {
        CONFLICTS,
        DELAY,
        NREWARDTYPES
    };
    RewardType _reward_type_mode;


    // CAPACITIES
    int get_flat_capacity() { return 2; }


    // AIRSPACE
    enum class AirspaceMode { SAVED, GENERATED };
    AirspaceMode _airspace_mode;

    // SUBCLASS MODES/CONSTANTS
    enum class TrafficMode { DETERMINISTIC, PROBABILISTIC };
    TrafficMode _traffic_mode;


    // UAV types

    enum class UAVType { SLOW, FAST, NTYPES = 1 };
    // const enum UAVType{SLOW,NTYPES};

    // CONSTANTS
    //! Returns 4 state elements for sectors (number of planes traveling in
    //! cardinal directions). Returns 1 for links.
    int get_n_state_elements() {
        if (_agent_defn_mode == UTMModes::AgentDefinition::SECTOR)
            return 4;
        else
            return 1;
    }
    int get_n_control_elements() {
        return get_n_state_elements()*get_n_types();
    }
    int get_n_steps() { return 200; }
    int get_n_types() { return static_cast<int>(UAVType::NTYPES); }
    double get_p_gen() { return 0.5; }

    //! UAVs are generated every get_gen_rate() steps
    int get_gen_rate() { return 10; }
    double get_dist_thresh() { return 2.0; }
    double get_conflict_thresh() { return 2.0; }
};


class UTMFileNames {

    bool numbered_domain;
    std::string domain_num;
    std::string gen_rate;
    std::string n_steps;
    std::string n_types;
    std::string n_sectors;
    std::string reward_mode;
    std::string alpha;
    std::string agent_defn;

public:

    explicit UTMFileNames(UTMModes* m) :
        numbered_domain(m->domain_num >= 0),
        domain_num(std::to_string(m->domain_num)),
        gen_rate(std::to_string(m->get_gen_rate())),
        n_steps(std::to_string(m->get_n_steps())),
        n_types(std::to_string(m->get_n_types())),
        n_sectors(std::to_string(m->get_n_sectors())),
        reward_mode(m->getRewardModeName()),
        alpha(std::to_string(static_cast<int>(m->alpha)))
    {
        switch (m->_agent_defn_mode) {
        case UTMModes::AgentDefinition::LINK:
            agent_defn = "Link";
            break;
        case UTMModes::AgentDefinition::SECTOR:
            agent_defn = "Sector";
            break;
        default:
            agent_defn = "Unknown";
            break;
        }
    }
    ~UTMFileNames() {}

    std::string createDomainDirectory() {
        std::string dir_path = "Domains/" + n_sectors + "_Sectors/";
        if (numbered_domain) dir_path += domain_num + "/";

        boost::filesystem::path dir(dir_path);
        boost::filesystem::create_directories(dir);
        return dir_path;
    }

    std::string createExperimentDirectory() {
        // Creates a directory for the experiment and returns that as a string

        std::string dir_path = "Experiments/"
            + agent_defn + "_Agents/"
            + n_sectors + "_Sectors/"
            + "Rate_" + gen_rate + "/"
            + n_steps + "_Steps/"
            + n_types + "_Types/"
            + reward_mode + "_Reward/"
            + alpha + "_alpha/";

            if (numbered_domain) dir_path += domain_num + "/";

            // Create new directory
            boost::filesystem::path dir(dir_path);
            boost::filesystem::create_directories(dir);

            return dir_path;
    }
};
#endif  // DOMAINS_UTM_UTMMODESANDFILES_H_
