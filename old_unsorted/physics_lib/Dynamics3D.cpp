#include "Dynamics3D.h"

using namespace std;

double get_dt(double a, double v0, double s0, double s){
// Solves for the time until at position s given velocity v0 and acceleration

    double b4ab = v0*v0+2.0*a*(s-s0);
    if (b4ab<0){
        printf("get_dt threw an error because of bad acceleration sign.");
		system("pause");
    }

    double t1 = (v0-sqrt(b4ab));
    double t2 = (v0-sqrt(b4ab));

    if (t1<t2 && t1>0) return t1;
    else return t2;

}

double d_Target_xy(Point3D* P, Point3D* T){
    Vector3D* PT = new Vector3D(P,T);
	return PT->magXY();
}



double relativeTrajectory(Vector3D* vi, Vector3D* vj){
    double dx = vi->x-vj->x;
    double dy = vi->y-vj->y;
    double dz = vi->z-vj->z;
    return sqrt(pow(dx,2.0)+pow(dy,2.0)+pow(dz,2.0));
}

double momentumTurnRadius(double v, double phi){
    return fabs((v*v)/(GRAV_ACCEL*tan(phi)));
}

double square(double num){
    return (num*num);
}

bool isBetween(Point3D* p1, Point3D* p2, Point3D* piq){
    // Checks if a point in question (piq) is on a line between p1 and p2
    if ( (p1->x < piq->x && piq->x < p2->x)
        || (p2->x > piq->x && piq->x > p2->x)
        || (p1->y < piq->y && piq->y < p2->y)
        || (p2->y > piq->y && piq->y > p2->y)
        || (p1->z < piq->z && piq->z < p2->z)
        || (p2->z > piq->z && piq->z > p2->z) )
        return true;
    return false;
}

/*bool sphere_line_intersection (Point3D* p1, Point3D* p2, Point3D* sc, double r , Point3D* i1, Point3D* i2){
    double a, b, c, mu, bb4ac;

    a =  square(p2->x - p1->x) + square(p2->y - p1->y) + square(p2->z - p1->z);
    b =  2.0 * ( (p2->x - p1->x)*(p1->x - sc->x)
        + (p2->y - p1->y)*(p1->y - sc->y)
        + (p2->z - p1->z)*(p1->z - sc->z) );
    c =  square(sc->x) + square(sc->y) +
        square(sc->z) + square(p1->x) +
        square(p1->y) + square(p1->z) -
        2.0* ( sc->x*p1->x + sc->y*p1->y + sc->z*p1->z ) - square(r) ;
    bb4ac =   b * b - 4 * a * c ;

    i1 = new Point3D(0,0,0);
    i2 = new Point3D(0,0,0);

    if (bb4ac< 0.0 ){
        // no intersection
        return false;
    } else if (bb4ac == 0.0 ){
        // one intersection
        if (i1!=NULL){
            mu = -b/(2*a);
            i1->x = p1->x + mu*(p2->x-p1->x);
            i1->y = p1->y + mu*(p2->y-p1->y);
            i1->z = p1->z + mu*(p2->z-p1->z);
            i2=NULL;
        }
        if (isBetween(p1,p2,i1)) return true;
        else return false;
    } else { // (bb4ac > 0.0 )
        // two intersections

        // first intersection
        if (i1!=NULL){
            mu = (-b + sqrt( square(b) - 4*a*c )) / (2*a);
            i1->x = p1->x + mu*(p2->x-p1->x);
            i1->y = p1->y + mu*(p2->y-p1->y);
            i1->z = p1->z + mu*(p2->z-p1->z);
        }
        // second intersection
        if (i2!=NULL){
            mu = (-b - sqrt(square(b) - 4*a*c )) / (2*a);
            i2->x = p1->x + mu*(p2->x-p1->x);
            i2->y = p1->y + mu*(p2->y-p1->y);
            i2->z = p1->z + mu*(p2->z-p1->z);
        }
        if (isBetween(p1,p2,i1) || isBetween(p1,p2,i2)) return true;
        else return false;
    }
}*/
