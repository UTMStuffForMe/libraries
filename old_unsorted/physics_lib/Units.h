#pragma once

#include <stdlib.h>
#include "../carrie_lib/printing.h"

#define KILOMETER 1000.0
#define MILLIMETER 1.0/1000.0
#define FEET 0.3048
#define INCH 0.0254

#define GRAM 1.0/1000.0 // Transforms grams to kilograms
#define POUND 0.453592
#define OUNCE 0.0283495

#define CUBIC_CENTIMETERS 0.001 // Transforms cubic centimeters to liters

#define MINUTES 60
#define HOUR 60*MINUTES

#define KILOMETER_PER_HOUR (1.0/3.6)
#define MILE_PER_HOUR 0.44704
#define KNOTS 0.514444444
#define FEET_PER_MINUTE 0.00508

#define PI 3.14159265
#define RAD (PI/(180.0))

// MKS UNITS (defined as 1, for explicit declaration)
#define METERS 1.0

bool isMKS(std::string unitname);
void convertToMKS(double *value, std::string unitname);
double mks(std::string value, std::string unit); // Returns the mks value for the value read
