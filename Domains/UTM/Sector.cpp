// Copyright 2016 Carrie Rebhuhn
#include "Sector.h"
#include <vector>

using easymath::XY;
using std::vector;

Sector::Sector(XY xy, int sectorIDSet, vector<int> connections,
    vector<XY> dest_locs, MultiGraph<LinkGraph>* highGraph, UTMModes* params) :
    xy(xy), ID(sectorIDSet), connections(connections){
}
