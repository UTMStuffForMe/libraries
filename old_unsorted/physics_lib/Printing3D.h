#pragma once

#include "Vector3D.h"
#include <utility>
#include <vector>
#include <string>
#include <iostream>

/*
| Prints gnuplot dat files of points.
*/

void printGnuPlotDatFile(std::vector<Point3D> * mypoints, std::string file_label="Unnamed_File.dat");
void printGnuPlotDatFile(std::vector<std::vector<Point3D*>* > * mypoints, std::string file_label="Unnamed_File.dat");
void printGnuPlotDatFile(std::vector<std::pair<int,int> > mypoints, std::string file_label="Unnamed_File.dat");
