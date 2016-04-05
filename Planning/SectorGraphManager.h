#pragma once

#include "../../libraries/Planning/GridGraph.h"
#include <map>

/**
* A manager class to handle different instances of grid for different sectors.
* Each 'type' in the simulation may have a different grid object (layer) assigned
* to it, in order to calculate traffic congestion differently and assign different
* costs to different UAV traffic types. This class stores the layer and manages
* access based on type.
*/

class SectorGraphManager
{
public:
	typedef std::pair<int,int> edge;

	SectorGraphManager(matrix2d membership_map, std::vector<edge> edges);
	~SectorGraphManager(void);

	int getMembership(const easymath::XY &p);
	std::vector<easymath::XY> astar(const easymath::XY &p1, const easymath::XY &p2);

private:
	matrix2d membership_map;
	std::map<int,std::map<int,GridGraph*> > m2graph; // lets you know which A* to access
};
