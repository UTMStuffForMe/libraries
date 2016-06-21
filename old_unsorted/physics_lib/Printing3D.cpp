#pragma once

#include "Printing3D.h"

using namespace std;

void printGnuPlotDatFile(vector<Point3D> *mypoints, string file_label){
    /*
    | Takes in a vector of points and prints out a dat file.
    */

    FILE * datFile;
    datFile = fopen(file_label.c_str(), "w");
    fprintf(datFile, "#\tX\tY\tZ\n");
    for (int i=0; i<mypoints->size(); i++){
        Point3D p = (*mypoints)[i];
        fprintf(datFile, "\t%f\t%f\t%f\n", p.x,p.y,p.z);
    }
    fclose(datFile);
}

void printGnuPlotDatFile(vector<vector<Point3D*> *> * mypoints, string file_label){
    /*
    | Takes in multiple vectors of points and prints out a dat file.
    */
    FILE * datFile;
    datFile = fopen(file_label.c_str(), "w");
    fprintf(datFile, "#\tX\tY\tZ\n");


    vector<vector<Point3D*> > animLog; // instead of a vector<vehicle logs>
                                        // it's a vector of vehicle positions
                                        // <v1(0),v2(0),v3(0)//v1(1),v2(1),v3(1) etc

    int maxLength = 0; // Maximum length of the vector
    for (int i=0; i<mypoints->size(); i++){
        int a = maxLength;
        int b = int((*mypoints)[i]->size());
        maxLength = (a<b)?b:a;
    }

    for (int i=0; i<maxLength; i++){ // i is the index to the point
        vector<Point3D*> animSnippet;
        for (int j=0; j<mypoints->size(); j++){ // j is the index to the log
            if ((*mypoints)[j]->size()>i){
                animSnippet.push_back((*(*mypoints)[j])[i]);
            }
        }
        animLog.push_back(animSnippet);
    }

    for (int i=0; i<animLog.size(); i++){
        for (int j=0; j<animLog[i].size(); j++){
            Point3D* p = animLog[i][j];
            fprintf(datFile, "\t%f\t%f\t%f\n", p->x,p->y,p->z);
        }
        fprintf(datFile,"\n\n"); // Adds a space between separate datasets.
    }

    /*for (int i=0; i<mypoints->size(); i++){
        for (int j=0; j<(*mypoints)[i]->size(); j++){
            Point3D* p = (*(*mypoints)[i])[j];
            fprintf(datFile, "\t%f\t%f\t%f\n", p->x,p->y,p->z);
        }
        fprintf(datFile,"\n"); // Adds a space between separate datasets.
    }*/

    fclose(datFile);
}

void printGnuPlotDatFile(vector<pair<int,int> > mypoints, string file_label){
    /*
    | Takes in one vector of int-int pairs and prints out a dat file.
    */
    FILE * datFile;
    datFile = fopen(file_label.c_str(), "w");
    fprintf(datFile, "#\tX\tY\n");
    for (int i=0; i<mypoints.size(); i++){
        int p1 = mypoints[i].first;
        int p2 = mypoints[i].second;
        fprintf(datFile,"\t%i\t%i\n",p1,p2);
    }
    fclose(datFile);
}
