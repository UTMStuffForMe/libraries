#pragma once

#include "Parser3D.h"

using namespace std;

void parseXYZ(bool *x, bool *y, bool *z, string direction){
    // Detects whether x, y, or z are in the string 'direction'
    *x = false;
    *y = false;
    *z = false;
    for (int i=0; i<direction.length(); i++){
        char ch = direction[i];
        if (ch=='x') *x=true;
        if (ch=='y') *y=true;
        if (ch=='z') *z=true;
    }
}
