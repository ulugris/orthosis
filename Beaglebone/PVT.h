#ifndef PVT_H
#define PVT_H

#include <QObject>
#include <QVector>

#define PI 3.14159265358979323846

class PVT
{
private:
    QVector<long> P;     // Position vector (qc)
    QVector<long> V;     // Velocity vector (rpm)
    QVector<int> T;      // Time intervals vector (ms)

    unsigned short ns;   // Number of time intervals
    double grt;          // Gear ratio
    double qpd;          // Quadcounts per degree at gear output

    double mv;           // Maximum velocity (rpm)
    double ma;           // Maximum acceleration (rpm)

public:
    PVT(const int nsteps = 6, const double ratio = 160.0, const int qpr = 4*1024);

    void set(const int nsteps, const double ratio, const int qpr);

    QVector<long> getP();
    QVector<long> getV();
    QVector<int> getT();

    void gen(double cl, const double ks, const double kw, const double kr);
    bool check(const double maxvel = 12500, const double maxacc = 1e6);
    void disp();
};

#endif // PVT_H
