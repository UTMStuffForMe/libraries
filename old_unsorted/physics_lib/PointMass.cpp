#include "PointMass.h"


PointMass::PointMass(double m, Point3D* p, Vector3D* o)
{
	// Defaults to unit everything
	mass = (m!=0) ? m : 1;
	position = (p!=NULL) ? p : Point3D(1,0,0);
	orientation = (o!=NULL) ? o: Vector3D(1,0,0);
}

void PointMass::updatePosition(double speed, double acceleration, double dt){
    position.x = nextPosition(acceleration*orientation.x, speed*orientation.x, position.x, dt);
    position.y = nextPosition(acceleration*orientation.y, speed*orientation.y, position.y, dt);
    position.z = nextPosition(acceleration*orientation.z, speed*orientation.z, position.z, dt);
}


// TURNING

void PointMass::xyTurn(double dt, double s_t, double a_t, double r, bool CCW){
    /*
    | Adjusts position and vector of travel in a turn of radius r.
    | Inputs:   dt - time increment     s_t - tangential speed
    |           r - radius of turn      p - 3D point in space
    |           v - vector of travel    CCW - true of turn is CCW, false if CW
    */

    // Angular velocity calculation
    double alpha_xy = (a_t/r)*orientation.magXY();
    double omega_xy = (s_t/r)*orientation.magXY();

    Point3D* turnCenterPoint = xyTurnCenter(CCW,r); // Locates the center of the turn

    // Adjusting the position/flightvector
    double angle_off = PI/2.0;
    if (!CCW){
        alpha_xy *= -1.0;
        omega_xy *= -1.0;
        angle_off *= -1.0;
    }
    double theta_OP = orientation.thetaXY()-angle_off; // Perpendicular vector from plane heading (vector from centerpoint of circle to position)

    double theta_OP_prime = nextPosition(alpha_xy,omega_xy,theta_OP,dt);
    position.x = turnCenterPoint->x+r*cos(theta_OP_prime);
    position.y = turnCenterPoint->y+r*sin(theta_OP_prime);
    position.z += orientation.z*s_t*dt;
    double theta_P_prime = theta_OP_prime+angle_off;
    orientation.x = cos(theta_P_prime);
    orientation.y = sin(theta_P_prime);
    orientation.unitScaleToZ(1.0); // Keeps the magnitude of the z-vector constant*/
}

Point3D* PointMass::xyTurnCenter(bool CCW, double r){
    /*
    | CCW = turn direction (true if CCW, false if CW)
    */
    double angle_off = PI/2.0;
    if (!CCW) angle_off *= -1.0;
    double theta_P = orientation.thetaXY();
    double theta_PO = theta_P+angle_off;
    double Ox = r*cos(theta_PO)+position.x;
    double Oy = r*sin(theta_PO)+position.y;
    return new Point3D(Ox,Oy,position.z);
}

double PointMass::distanceToPointXY(bool CCW, Point3D* T, double r){
    /*
    | CCW = turn direction (true if CCW, else false)
    | T = target position
    | r = turn radius
    | returns: the magnitude of the xy-distance to the target.
    */

    Point3D* O = xyTurnCenter(CCW,r); // Center of the turn
    Vector3D* OT_hat = new Vector3D(O,T); // Vector from the center of the circle to the target
	double OT = OT_hat->magXY();
    double alpha = acos(r/OT);
    double TP_prime = sqrt(pow(OT,2)+pow(r,2));

    Vector3D* OP_hat = new Vector3D(O,&position); // Vector from the center of the circle to the plane
    double beta = fabs(angleXY(OP_hat,OT_hat));
    double theta_P_P_prime = 2*PI - alpha - beta;

    return TP_prime + theta_P_P_prime*r;
}


// GLOBAL functions

double nextPosition(double a, double v0, double s0, double t){
    /*
    | Returns a position based on acceleration, velocity, starting position, and time increment.
    | For angular position, substitute angular acceleration, angular velocity, and starting angle.
    */
    return s0 + v0*t + 0.5*a*t*t;
}

Point3D nextPosition(Point3D p, Vector3D o, double a, double v0, double t){
	Point3D nextPos;
	nextPos.x = nextPosition(a*o.x,v0*o.x,p.x, t);
	nextPos.y = nextPosition(a*o.y,v0*o.y,p.y, t);
	nextPos.z = nextPosition(a*o.z,v0*o.z,p.z, t);
	return nextPos;
}

double distanceManeuver(Point3D position, Point3D detour, Point3D goal){
	// Returns EXTRA cost caused by going to the detour point

	double distanceDirect = distance3D(position, goal);
	double distancePositionToDetour = distance3D(position, detour);
	double distanceDetourToGoal = distance3D(detour, goal);
	return distancePositionToDetour+distanceDetourToGoal-distanceDirect;
}		