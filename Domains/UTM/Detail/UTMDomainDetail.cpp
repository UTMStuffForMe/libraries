// Copyright 2016 Carrie Rebhuhn
#include "UTMDomainDetail.h"
#include <string>
#include <list>
#include <vector>

using std::vector;
using easymath::XY;
using easymath::zeros;
using std::list;

UTMDomainDetail::UTMDomainDetail(UTMModes* params_set) :
    UTMDomainAbstract(params_set){

    matrix2d membership_map =
        FileIn::read2<double>("agent_map/membership_map.csv");

    // Planning
    lowGraph = MultiGraph<GridGraph>(highGraph()->get_n_edges(), GridGraph(membership_map));

    vector<XY> sector_locs(sectors.size());
    for (int i = 0; i < sectors.size(); i++) {
        sector_locs[i] = sectors[i]->xy;
    }
    for (Sector* s : sectors) {
        s->generation_pt = FixDetail(s->xy, s->ID, highGraph, lowGraph, sector_locs, params, linkIDs);
    }
}

UTMDomainDetail::~UTMDomainDetail(void) {}


void UTMDomainDetail::logUAVLocations() {
    matrix1d stepLocation;
    for (UAV* u : UAVs) {
        stepLocation.push_back(static_cast<UAVDetail*>(u)->loc.x);
        stepLocation.push_back(static_cast<UAVDetail*>(u)->loc.y);
    }
    UAVLocations.push_back(stepLocation);
}


void UTMDomainDetail::exportUAVLocations(int fileID) {
    FileOut::print_vector(UAVLocations,
        "visualization/locations" + std::to_string(fileID) + ".csv");
}

vector<double> UTMDomainDetail::getPerformance() {
    // TODO
    return zeros(1);
}


vector<double> UTMDomainDetail::getRewards() {
    // TODO

    // DELAY REWARD
    return zeros(1);
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
    return lowGraph()->get_membership(p);
}

// HACK: ONLY GET PATH PLANS OF UAVS just generated
void UTMDomainDetail::getPathPlans() {
    // REPLACE WITH PLANPATH
    for (UAV* u : UAVs) {
        // sets own next waypoint
        static_cast<UAVDetail*>(u)->planDetailPath();
    }
}

void UTMDomainDetail::getPathPlans(const std::list<UAV* > &new_UAVs) {
    for (UAV* u : new_UAVs) {
        static_cast<UAVDetail*>(u)->planDetailPath();  // sets own next waypoint
    }
}


void UTMDomainDetail::incrementUAVPath() {
    for (UAV* u : UAVs) {
        // moves toward next waypoint (next in low-level plan)
        static_cast<UAVDetail*>(u)->moveTowardNextWaypoint();
    }
}

void UTMDomainDetail::reset() {
    UAVs.clear();
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
