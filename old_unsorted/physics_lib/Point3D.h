#pragma once

#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include "..\carrie_lib\shortcuts.h"

#define PI 3.14159265

class Point3D{
    public:
	~Point3D(void){}; // DEFAULT
    Point3D(void){}; // DEFAULT
    Point3D(double cubeSize); // Creates a within a cubical (positive) world
    Point3D(double xi, double yi, double zi); // Creates a point with specified x,y,z
    Point3D(Point3D* other); // Copies a point given a pointer
    void operator=(Point3D* other);
	Point3D& operator+(Point3D &other);
	Point3D& operator/(double d);
	bool operator==(Point3D &other);
	bool operator!=(Point3D &other);

	double x, y, z;
};

void show(Point3D* p);
double distance3D(Point3D* p1, Point3D* p2);
double distance3D(Point3D p1, Point3D p2);
Point3D midpoint(Point3D p1, Point3D p2);
double distanceXY(Point3D p1, Point3D p2);