#pragma once

#include "Vector3D.h"
#include <vector>

class Sphere3D{
    public:
    Sphere3D(Point3D* c, double r); // NOTE: CHECK AND SEE IF THIS TRACKS WITH POSITION WHEN IMPLEMENTING WITH VEHICLES

    Point3D* center; // Centerpoint of the sphere
    double radius; // Radius of the sphere

    bool intersects(Sphere3D* other); // Detects intersection with another sphere
    std::vector<Point3D> draw(int increments); // Draws the sphere

    private:
    Point3D getPoint(Vector3D* normal, double theta);
};