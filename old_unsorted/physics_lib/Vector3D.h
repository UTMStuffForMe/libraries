#pragma once

#include "Point3D.h"

class Vector3D{
    public:
    double x, y, z;

    // Initialization Functions
    Vector3D();
    Vector3D(Vector3D* v);
    Vector3D(double xi, double yi, double zi);
    Vector3D(Point3D* p1, Point3D* p2);
	Vector3D(Point3D p1, Point3D p2);

    // Attribute Calculation
    double mag();
    double magXY();
    double thetaXY();

    // Scaling
    void makeUnit();
    void unitScaleToZ(double desired_magnitude);
	
	// Adjustment
	void rotateXY(double angle);

    // Operators
    Vector3D operator*(const double& scalar);
    double operator*(Vector3D& other);
    void operator=(Vector3D other);
};

Point3D endPoint(Point3D start, Vector3D orientation, double dist);
void show(Vector3D* v);
double angleXY(Vector3D* v1, Vector3D* v2);
