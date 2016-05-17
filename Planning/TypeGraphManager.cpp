// Copyright 2016 Carrie Rebhuhn
#include "TypeGraphManager.h"

#include <list>
#include <string>
#include <algorithm>
#include <vector>
#include <set>

using std::vector;
using easymath::XY;
using std::list;
using std::string;
using std::set;
using easymath::line_segment;
using easymath::get_n_unique_square_points;
using std::make_pair;

TypeGraphManager::TypeGraphManager(void) {
}

TypeGraphManager::TypeGraphManager(int n_types, vector<edge> edges,
    vector<XY> locs) :
    n_types(n_types), edges(edges), rags_map(new RAGS(locs, edges)) {
    initializeTypeLookupAndDirections(locs);
}

TypeGraphManager::TypeGraphManager(string efile, string vfile, int n_types) :
    n_types(n_types),
    edges(FileIn::read_pairs<edge>(efile)) {
    // NOTE: this leaves to the user the task of making edges bidirectional
    // Read in files for sector management
    vector<XY> agentLocs = FileIn::read_pairs<XY>(vfile);
    rags_map = new RAGS(agentLocs, edges);

    for (size_t i = 0; i < agentLocs.size(); i++) {
        loc2mem[agentLocs[i]] = i;  // add in reverse lookup
    }

    initializeTypeLookupAndDirections(agentLocs);
}

TypeGraphManager::TypeGraphManager(int n_vertices, int n_types,
    double xdim, double ydim) :
    n_types(n_types) {
    // set<XY> agent_loc_set
    // = get_n_unique_points(0.0,gridSizeX,0.0,gridSizeY, n_vertices);
    set<XY> agent_loc_set
        = get_n_unique_square_points(0.0, xdim, 0.0, ydim, n_vertices);
    vector<XY> agentLocs(agent_loc_set.size());
    copy(agent_loc_set.begin(), agent_loc_set.end(), agentLocs.begin());

    set<edge> candidates_set;
    for (size_t i = 0; i < agentLocs.size(); i++) {
        for (size_t j = 0; j < agentLocs.size(); j++) {
            if (i == j)
                continue;
            candidates_set.insert(make_pair(i, j));
        }
    }
    vector<edge > candidates(candidates_set.size());
    copy(candidates_set.begin(), candidates_set.end(), candidates.begin());

    random_shuffle(candidates.begin(), candidates.end());


    set<edge> edges_set;
    // Add as many edges as possible
    for (edge c : candidates) {
        if (!intersectsExistingEdge(c, agentLocs,edges_set)) {
            edges_set.insert(c);
            edges_set.insert(std::make_pair(c.second, c.first));
            //edges.push_back(c);
            //edges.push_back(std::make_pair(c.second, c.first));
        }
    }
    edges = vector<edge>(edges_set.size());
    copy(edges_set.begin(), edges_set.end(), edges.begin());

    // create a RAGS object which generates a graph for searching over
    rags_map = new RAGS(agentLocs, edges);

    for (size_t i = 0; i < agentLocs.size(); i++) {
        loc2mem[agentLocs[i]] = i;  // add in reverse lookup
    }

    initializeTypeLookupAndDirections(agentLocs);
}


bool TypeGraphManager::intersectsExistingEdge(edge candidate, vector<XY> locs, set<edge> edges_set) {
    for (edge e : edges_set) {
        line_segment l1, l2;
        l1 = line_segment(locs[e.first], locs[e.second]);
        l2 = line_segment(locs[candidate.first], locs[candidate.second]);
        if (easymath::intersects_in_center(l1,l2))
            return true;
    }
    // check ALSO if any agent locations are being crossed by an edge
    for (XY a : locs) {
        XY e1 = locs[candidate.first];
        XY e2 = locs[candidate.second];
        if (a == e1 || a == e2)
            continue;

        // CHECK IF POINT ON SEGMENT
        double m = (e2.y - e1.y) / (e2.x - e1.x);
        double b = e1.y - e1.x*m;

        double threshold = 0.01;
        double y_on_line = m*a.x + b;
        double diff = fabs(a.y - y_on_line);
        bool not_on_line = diff > threshold;

        bool abv_xmin = a.x >= std::min(e1.x, e2.x);
        bool blw_xmax = a.x <= std::max(e1.x, e2.x);
        bool abv_ymin = a.y >= std::min(e1.y, e2.y);
        bool blw_ymax = a.y <= std::max(e1.y, e2.y);
        if (not_on_line) {
            continue; // not on line, check next one
        } else if (abv_xmin && blw_xmax && abv_ymin && blw_ymax) {
            return true;    // on the line segment!
        }
    }
    return false;
}

bool TypeGraphManager::fullyConnected(vector<XY> agentLocs) {
    LinkGraph a = LinkGraph(agentLocs, edges);

    for (size_t i = 0; i < agentLocs.size(); i++) {
        for (size_t j = 0; j < agentLocs.size(); j++) {
            if (i == j) continue;
            list<int> path = a.astar(i, j);
            if (path.size() == 1)
                return false;
        }
    }
    return true;
}


TypeGraphManager::~TypeGraphManager(void) {
    delete rags_map;
    for (int i = 0; i < n_types; i++) {
        delete Graph_highlevel[i];
    }
}

void TypeGraphManager::setCostMaps(matrix2d agent_actions) {
    for (size_t i = 0; i < Graph_highlevel.size(); i++) {
        Graph_highlevel[i]->setWeights(agent_actions[i]);
    }
}


list<int> TypeGraphManager::astar(int mem1, int mem2, int type_ID) {
    return Graph_highlevel[type_ID]->astar(mem1, mem2);
}

list<int> TypeGraphManager::rags(int mem1, int mem2, int type_ID) {
    matrix1d w = Graph_highlevel[type_ID]->getWeights();
    XY start_loc = getLocation(mem1);
    XY end_loc = getLocation(mem2);
    XY next_xy = rags_map->SearchGraph(start_loc, end_loc, w);
    int next_node_ID = getMembership(next_xy);
    list<int> partial_path;
    partial_path.push_back(mem1);  // Add in starting node too
    partial_path.push_back(next_node_ID);
    // NOTE TO RYAN: debug may be needed here.
    // RAGS doesn't always seem to find a path.
    // Please make sure I'm using it correctly.s
    return partial_path;
}

int TypeGraphManager::getMembership(easymath::XY pt) {
    try {
        return loc2mem.at(pt);
    }
    catch (std::out_of_range) {
        printf("Point (%f,%f) not found in membership lookup. ", pt.x, pt.y);
        printf("Were you trying to use the detailed map ?\n");
        system("pause");
        exit(1);
    }
}

XY TypeGraphManager::getLocation(int sectorID) {
    return Graph_highlevel[0]->locations[sectorID];
}

vector<TypeGraphManager::edge> TypeGraphManager::getEdges() {
    return edges;
}

int TypeGraphManager::getNVertices() {
    return Graph_highlevel[0]->locations.size();
}

void TypeGraphManager::initializeTypeLookupAndDirections(vector<XY> agentLocs) {
    Graph_highlevel = std::vector<LinkGraph*>(n_types);
    for (int i = 0; i < n_types; i++) {
        Graph_highlevel[i] = new LinkGraph(agentLocs, edges);
    }
}
