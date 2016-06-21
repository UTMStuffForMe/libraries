#pragma once

class Spring{
    // MODEL FOR A BIDIRECTIONAL SPRING
    public:
    Spring(double spring_constant, double equilibrium_length);
    double force(double spring_length);
    bool inCompression(double spring_length);

    double k;
    double l;
};
