#include "Units.h"
#include <cstring>
using namespace std;

bool isMKS(string unitname){
    /*
    | Checks to determine if unit is MKS or not.
    */
    if (strcmp(unitname.c_str(),"m")) return true;
    else if (strcmp(unitname.c_str(),"kg")) return true;
    else if (strcmp(unitname.c_str(),"s")) return true;
    else if (strcmp(unitname.c_str(),"m/s")) return true;
    else if (strcmp(unitname.c_str(),"m/s^2")) return true;
    else if (strcmp(unitname.c_str(),"l")) return true;
    else return false;
}

void convertToMKS(double *value, string unitname){
    /*
    | Converts a given value to its MKS counterpart.
    */

    if (isMKS(unitname)) return;

    // DISTANCE
    if (strcmp(unitname.c_str(),"km")) (*value) *= KILOMETER;
    else if (strcmp(unitname.c_str(),"ft")) (*value) *= FEET;
    else if (strcmp(unitname.c_str(),"mm")) (*value) *= MILLIMETER;
    else if (strcmp(unitname.c_str(),"in")) (*value) *= INCH;
    // VOLUME
    else if (strcmp(unitname.c_str(),"cc")) (*value) *= CUBIC_CENTIMETERS;
    // SPEED
    else if (strcmp(unitname.c_str(),"km/h")) (*value) *= KILOMETER_PER_HOUR;
    else if (strcmp(unitname.c_str(),"mph")) (*value) *= MILE_PER_HOUR;
    else if (strcmp(unitname.c_str(),"kt")) (*value) *= KNOTS;
    else if (strcmp(unitname.c_str(),"ft/min")||strcmp(unitname.c_str(),"fpm")) (*value) *= FEET_PER_MINUTE;
    //WEIGHT
    else if (strcmp(unitname.c_str(),"g")) (*value) *= GRAM;
    else if (strcmp(unitname.c_str(),"lb")) (*value) *= POUND;
    else if (strcmp(unitname.c_str(),"oz")) (*value) *= OUNCE;
    // ANGLE
    else if (strcmp(unitname.c_str(),"degrees")) (*value) *= RAD;
    else {
        printf("Unknown unit %s: please add unit definition to physics_lib/Units.h or edit config file.\n",unitname.c_str());
        wait_for_key();
    }
}

double mks(string value, string unit){
    /*
    | Converts a value with a unit to a double value in mks.
    */
    double myval = atof(value.c_str());
    convertToMKS(&myval,unit);
    return myval;
}
