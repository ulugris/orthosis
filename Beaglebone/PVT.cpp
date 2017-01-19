#include <cmath>
#include <iostream>
#include <iomanip>

#include "PVT.h"

// PVT constructor
PVT::PVT(const int nsteps, const double ratio, const int qpr)
{
    set(nsteps, ratio, qpr);
}

// Set PVT array basic parameters
void PVT::set(const int nsteps, const double ratio, const int qpr)
{
    ns = nsteps;
    grt = ratio;
    qpd = qpr*grt/360;
}

// Return position array
QVector<long> PVT::getP()
{
    return P;
}

// Return velocity array
QVector<long> PVT::getV()
{
    return V;
}

// Return time array
QVector<int> PVT::getT()
{
    return T;
}

// Generate PVT array from parameter set
//   cl: cycle length (s)
//   ks: peak displacement parameter (non-dimensional)
//   kw: peak width parameter (non-dimensional)
//   kr: maximum knee flexion angle (deg)
void PVT::gen(double cl, const double ks, const double kw, const double kr)
{
    // Round interval length to nearest millisecond
    double dt = round(cl/ns*1000)/1000;
    cl = ns*dt;

    P.resize(0);
    V.resize(0);
    T.resize(0);

    for (int i = 0; i < ns; i++)
    {
        double t = dt*(i + 1);

        double wt = 2*PI/cl*t;
        double ph = ks*sin(wt/2) + kw*sin(wt);

        double pos = kr/2*(1 - cos(wt - ph));
        double vel = kr*PI/cl*(sin(wt - ph)*(1 - ks/2*cos(wt/2)- kw*cos(wt)));

        P.append(round(pos*qpd));
        V.append(round(vel*grt/6));
        T.append(round(dt*1000));
    }

    P.append(0);
    V.append(0);
    T.append(0);
}

// Check curve feasibility (maxvel in rpm, maxacc in rpm/s)
bool PVT::check(const double maxvel, const double maxacc)
{
    mv = 0.0;
    ma = 0.0;

    double p0 = 0.0; // Initial position
    double v0 = 0.0; // Initial velocity

    for (int i = 0; i < ns; i++)
    {
        // Interval length must be between 1 and 255 ms
        if (T[i] > 255 || (i < ns-1 && T[i] <= 0))
            return false;

        double p1 = P[i]/qpd;    // Output position at interval end (deg)
        double v1 = V[i]*6/grt;  // Output velocity at interval end (deg/s)
        double dt = T[i]/1000.0; // Current interval length (s)

        // Polynomial coefficients
        double a = (2*(p0 - p1) + dt*(v0 + v1))/pow(dt, 3);
        double b = (3*(p1 - p0) - dt*(2*v0 + v1))/pow(dt, 2);

        // Null acceleration point
        double t0 = -b/(3*a);

        // Update maximum velocity
        if (abs(v1) > mv)
            mv = abs(v1);
        if (t0 > 0 && t0 < dt && abs(v0 + b*t0) > mv)
            mv = abs(v0 + b*t0);

        // Update maximum acceleration
        if (abs(2*b) > ma)
            ma = abs(2*b);
        if (abs(2*b + 6*a*dt) > ma)
            ma = abs(2*b + 6*a*dt);

        p0 = p1;
        v0 = v1;
    }

    mv = mv*grt/6; // Convert output velocity to motor rpm
    ma = ma*grt/6; // Convert output acceleration to motor rpm/s

    if (mv > maxvel || ma > maxacc)
        return false;
    else
        return true;
}

// Display PVT array in EPOS2 units
void PVT::disp()
{
    for (int i = 0; i < T.size(); i++)
    {
        std::cout << std::setw(9) << P[i] << std::setw(7) << V[i];
        std::cout << std::setw(4) << T[i] << std::endl;
    }
}
