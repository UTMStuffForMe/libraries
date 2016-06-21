#pragma once

#include "Spring.h"

Spring::Spring(double spring_constant, double equilibrium_length){
    k=spring_constant;
    l=equilibrium_length;
}

double Spring::force(double spring_length){
    /*
    | Returns a force according to Hooke's Law.
    */
    return -k*(spring_length-l);
}

bool Spring::inCompression(double spring_length){
    /*
    | Returns true if in compression, else false.
    */
    return (l>spring_length);
}
