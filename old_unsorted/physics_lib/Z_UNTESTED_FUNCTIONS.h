#pragma once

double abs_angle(double angle_rads){ // Converts angle to be between +- pi
    int revs = angle_rads/(2*PI);
    double a = angle_rads - (double)revs*2*PI;
	if (a<=PI) return a;
	else return a - 2*PI;
}

void m_multiply(double matrix3x1[3], double scalar){
    for (int i=0; i<3; i++) matrix3x1[i]*=scalar;
}

void scaleToUnit(unitvector myvector){
    m_multiply(myvector, 1/mag(myvector));
}

void setMatrix(transform3D m1, transform3D m2){ // Set m1=m2, for 3D transforms
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            m1[i][j]=m2[i][j];
}

void setMatrix(position m1, position m2){ // Set m1=m2, for positions
    for (int i=0; i<3; i++) m1[i]=m2[i];
}

void m_multiply(transform3D R, position positionToTransform){ // Matrix multiplies a 3D rotation matrix and a position
    position init;
    setMatrix(init, positionToTransform);
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            positionToTransform[i] += init[i]*R[i][j];
}

void translate(position origin_absolute, position transformToRelative){ // Translates a point according to the absolute of the reference position
    for (int i=0; i<3; i++) transformToRelative[i] -= origin_absolute[i];
}

void R_x(double alpha, transform3D xRotationMatrix){ // Converts xRotationMatrix to the correct unit rotation matrix
    transform3D tempMatrix =
    {{1,0,0},
    {0,cos(alpha),sin(alpha)},
    {0,-sin(alpha),cos(alpha)}};
    setMatrix(xRotationMatrix,tempMatrix);
}

void R_y(double beta, transform3D yRotationMatrix){
    transform3D tempMatrix =
    {{cos(beta),0,-sin(beta)},
    {0,1,0},
    {sin(beta),0,cos(beta)}};
    setMatrix(yRotationMatrix,tempMatrix);
}

void R_z(double gamma, transform3D zRotationMatrix){
    transform3D tempMatrix =
    {{cos(gamma),sin(gamma),0},
    {-sin(gamma),cos(gamma),0},
    {0,0,1}};
    setMatrix(zRotationMatrix,tempMatrix);
}

void rotate_x(double alpha, position positionToTransform){ // Performs coordinate rotation
    transform3D Rx;
    R_x(alpha, Rx);
    m_multiply(Rx, positionToTransform);
};

void rotate_y(double beta, position positionToTransform){ // Performs coordinate rotation
    transform3D Ry;
    R_y(beta, Ry);
    m_multiply(Ry, positionToTransform);
};

void rotate_z(double gamma, position positionToTransform){ // Performs coordinate rotation
    transform3D Rz;
    R_z(gamma, Rz);
    m_multiply(Rz, positionToTransform);
};

void transformCoordinatesByNormal(unitvector normal, position origin, position absoluteToRelative){
    // Performs transformations to align z to plane normal (for 3D computation)
    double alpha = atan(normal[2]/normal[1])-HALF_PI;
    double beta = atan(normal[2]/normal[0]-HALF_PI);
    double gamma = atan(normal[1]/normal[0]-HALF_PI);

    rotate_z(gamma, absoluteToRelative);
    rotate_y(beta, absoluteToRelative);
    rotate_x(alpha, absoluteToRelative);
    translate(origin, absoluteToRelative);
}

void untransformCoordinatesByNormal(unitvector normal, position origin, position relativeToAbsolute){
    double alpha = atan(normal[2]/normal[1])-HALF_PI;
    double beta = atan(normal[2]/normal[0]-HALF_PI);
    double gamma = atan(normal[1]/normal[0]-HALF_PI);

    position inversePosition;
    for (int i=0; i<3; i++) inversePosition[i] = -origin[i];

    translate(inversePosition, relativeToAbsolute);
    rotate_x(-alpha, relativeToAbsolute);
    rotate_y(-beta, relativeToAbsolute);
    rotate_z(-gamma, relativeToAbsolute);
}

#endif
