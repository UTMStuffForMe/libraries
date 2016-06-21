#pragma once

#include "Sphere3D.h"

using namespace std;

Sphere3D::Sphere3D(Point3D* c, double r){
    center = c;
    radius = r;
}

bool Sphere3D::intersects(Sphere3D* other){
    Vector3D* centerToCenter = new Vector3D(center,other->center);
    if (centerToCenter->mag()>radius+other->radius) return false;
    else return true;
}

vector<Point3D> Sphere3D::draw(int n_increments){
    /*
    | Returns a vector of points of resolution DPR (dots per rad, in every direction)
    */
    // DRAW CIRCLE
    // TILT AROUND Z-AXIS
    // REDRAW CIRCLE ON NEW PLANE

    vector<Point3D> spherePoints;
    /*Vector3D* norm = new Vector3D(1,0,0);
    double theta=0.0; // Angle of rotation
    double dTheta = PI/(double)n_increments; // Change in angle each time
    for (int i=0; i<n_increments; i++){
        vector<Point3D> circlePoints = draw(norm,n_increments);
        spherePoints.insert(spherePoints.end(),circlePoints.begin(),circlePoints.end());

        theta+=dTheta;
        norm->x+=norm->mag()*cos(theta); // Clockwise rotation about z-axis
        norm->y+=norm->mag()*sin(theta);
    }

	*/

	return spherePoints;
}

Point3D Sphere3D::getPoint(Vector3D* norm, double theta){
    // Gets point on xy-plane, then translates to z based on normal rotation and translation
    double xpoint = radius*cos(theta);
    double ypoint = radius*sin(theta);
    double zpoint = 0.0;

    // TODO: Put on a plane defined by the normal and the centerpoint
    Vector3D* refnorm = new Vector3D(0,0,1); // Normal of the coordinates in which the points were calced

    //TODO: finish this

    // Translate to absolute coordinates
    xpoint+=center->x;
    ypoint+=center->y;
    zpoint+=center->z;
    return *(new Point3D(xpoint, ypoint, zpoint));
}