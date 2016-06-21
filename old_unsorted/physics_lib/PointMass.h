#pragma once
#include "Vector3D.h"
#include "Point3D.h"

class PointMass
{
public:
	PointMass(double m=0, Point3D* p=NULL, Vector3D* o=NULL);
	~PointMass(void){}; // No delete functions necessary because actual objects used
	double mass;
	Point3D position; // Instantaneous position
	Vector3D orientation; // NOTE: this is instantaneous orientation (does not include velocity)

	void updatePosition(double speed, double acceleration, double dt); //Updates the position
	void xyTurn(double dt, double s_t, double a_t, double r, bool CCW);
	double distanceToPointXY(bool CCW, Point3D* T, double r);

private:
	Point3D* xyTurnCenter(bool CCW, double r);
};

// Dynamics dealing with point masses:
double nextPosition(double a, double v0, double s0, double t);
Point3D nextPosition(Point3D p, Vector3D o, double a, double v0, double t);
double distanceManeuver(Point3D position, Point3D detour, Point3D goal);		