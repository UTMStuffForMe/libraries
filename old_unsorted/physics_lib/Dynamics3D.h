#pragma once
#define GRAV_ACCEL 9.81 // m/s/s

#include "Vector3D.h"

void xyTurn(double dt, double s_t, double a_t, double r, Point3D* p, Vector3D* v, bool CCW);
double d_Target_xy(Point3D* P, Point3D* T);
double d_Target_xy(Vector3D* P_hat, Point3D* P, bool CCW, Point3D* T, double r);
Point3D* xyTurnCenter(Vector3D* P_hat, Point3D* P, bool CCW, double r);
double momentumTurnRadius(double v, double phi);

/**
* Takes two points defining a line segment, a sphere center, and a radius. Optionally
* changes mu1 and mu2 to be the points of intersection on the sphere (default NULL)
* Adapted from code by Paul Bourke at http://paulbourke.net/geometry/sphereline/
* @param[in] p1,p2 Points defining a line segment
* @param[in] sc A centerpoint for the sphere
* @param[in] r The radius of the sphere
* @param[out] i1,i2 The intersection points (x-values? TODO: needs a double-check)
*/
//bool sphere_line_intersection (Point3D* p1, Point3D* p2, Point3D* sc, double r , Point3D* i1=NULL, Point3D* i2=NULL);
