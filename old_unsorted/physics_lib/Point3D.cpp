#pragma once

#include "Point3D.h"

Point3D::Point3D(double cubeSize){
    /*
    | Creates a random point within the bounds of a cube.
    */
	x = cubeSize*bRand();
    y = cubeSize*bRand();
    z = cubeSize*bRand();
}

Point3D::Point3D(double xi, double yi, double zi){
    /*
	| Creates a point with the specified parameters.
	*/
	x=xi;
	y=yi;
	z=zi;
}

Point3D::Point3D(Point3D* other){
    x=other->x;
	y=other->y;
	z=other->z;
}

void Point3D::operator=(Point3D* other){
    x=other->x;
	y=other->y;
	z=other->z;
}

bool Point3D::operator==(Point3D &other){
    return (x==other.x && y==other.y && z==other.z);
}

bool Point3D::operator!=(Point3D &other){
	return !(*this == other);
}

Point3D& Point3D::operator+(Point3D &other){
	x+=other.x;
	y+=other.y;
	z+=other.z;
	return *this;
}

Point3D& Point3D::operator/(double d){
	x/=d;
	y/=d;
	z/=d;
	return *this;
}

double distance3D(Point3D* p1, Point3D* p2){
    double dx = p1->x-p2->x;
    double dy = p1->y-p2->y;
    double dz = p1->z-p2->z;
    return sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2));
}

double distance3D(Point3D p1, Point3D p2){
    double dx = p1.x-p2.x;
    double dy = p1.y-p2.y;
    double dz = p1.z-p2.z;
    return sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2));
}

void show(Point3D* p){
    printf("(%f,%f,%f)\n",p->x,p->y,p->z);
}

Point3D midpoint(Point3D p1, Point3D p2){
	Point3D p;
	p = p1+p2;
	p = p/2.0;
	return p;
}


double distanceXY(Point3D p1, Point3D p2){
	double dx = p1.x - p2.x;
	double dy = p1.y - p2.y;
	return sqrt(dx*dx+dy*dy);
}