#pragma once

#include "Vector3D.h"

using namespace std;


// Initialization

Vector3D::Vector3D(){
    /*
    | Creates a unit vector with random orientation.
    */
    x=rand();
	y=rand();
	z=rand();
    makeUnit();
}

Vector3D::Vector3D(Vector3D* v){
    x=v->x;
	y=v->y;
	z=v->z;
}

Vector3D::Vector3D(double xi, double yi, double zi){
    /*
    | Creates a vector <xi,yi,zi>.
    */
    x=xi;
	y=yi;
	z=zi;
}

Vector3D::Vector3D(Point3D* p1, Point3D* p2){
    /*
    | Creates a vector from p1 to p2.
    */
    x=(p2->x-p1->x);
	y=(p2->y-p1->y);
	z=(p2->z-p1->z);
}

Vector3D::Vector3D(Point3D p1, Point3D p2){
    x=(p2.x-p1.x);
	y=(p2.y-p1.y);
	z=(p2.z-p1.z);
}

// Attribute Calculation

double Vector3D::magXY(){
    /*
    | Returns the magnitude of the vector in the xy-direction.
    */
    return sqrt(pow(x,2)+pow(y,2));
}

double Vector3D::mag(){
    /*
    | Returns the magnitude of the vector.
    */
    return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
}

double Vector3D::thetaXY(){
    /*
    | Returns the angle of the vector off of the x-axis (+- PI) on the xy-plane.
    */
    if (x==0 && y>0) return 0.5*PI;
    if (x==0 && y<0) return -0.5*PI;
    double theta = atan2(y,x);
    return theta;
}

// Comparison Functions


// Scaling

void Vector3D::makeUnit(){
    /*
    | Converts the vector to a unit vector.
    */
    double m = mag();
    if (m!=0) x=x/m, y=y/m, z=z/m;
}

void Vector3D::unitScaleToZ(double desired_magnitude){
    /*
    | Inputs: Desired magnitude for the non-unit vector
    | Functions: Changes the x and y element to get to that magnitude (z constant)
    | Returns unit vector with the desired proportions
    */
    double num = pow(desired_magnitude,2)-pow(z,2);
    if (num<0){
        printf("Error in unitScaleToZ: invalid magnitude.\n");
    }
    double denom = 1.0+pow(y/x,2);
    double xnew;
    if (x>0) xnew = sqrt(num/denom);
    else xnew = -sqrt(num/denom);
    double ynew = (y/x)*xnew;
    x = xnew, y = ynew;
    makeUnit();
}

void Vector3D::rotateXY(double dTheta){
	double thetaPrime = thetaXY() + dTheta;
	x = cos(thetaPrime);
	y = sin(thetaPrime);
}

// Operators

Vector3D Vector3D::operator*(const double& scalar){
    /*
    | Scalar multiplication of the vector.
    */
    double xi=x*scalar, yi=y*scalar, zi=z*scalar;
    return Vector3D(xi,yi,zi);
}

double Vector3D::operator*(Vector3D& other){
    /*
    | Returns the dot product of the vectors.
    */
    return x*other.x + y*other.y + z*other.z;
}

void Vector3D::operator=(Vector3D other){
    /*
    | Sets the vector equal to the given vector.
    */
    x=other.x, y=other.y, z=other.z;
}

// Global functions

double angleXY(Vector3D* v1, Vector3D* v2){
    /*
    | Returns the xy-planar angle (+- PI) to the other vector.
    */
    double xy1 = v1->thetaXY();
    double xy2 = v2->thetaXY();
    double dxy = xy2-xy1;
    if (fabs(dxy)>PI && dxy>0) return dxy-2*PI;
    else if (fabs(dxy)>PI && dxy<0) return dxy+2*PI;
    else return dxy;
}

void show(Vector3D* v){
    /*
    | Prints out the vector.
    */
    printf("<%f,%f,%f>\n",v->x,v->y,v->z);
}

Point3D endPoint(Point3D start, Vector3D orientation, double dist){
	Point3D retPoint;
	retPoint.x = start.x + orientation.x*dist;
	retPoint.y = start.y + orientation.y*dist;
	retPoint.z = start.z + orientation.z*dist;
	return retPoint;
}