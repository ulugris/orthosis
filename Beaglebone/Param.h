#ifndef PARAM_H
#define PARAM_H

#include <QVector>
#include <fstream>
#include <memory>

#include "PVT.h"

#define MAXVEL 12500
#define MAXACC 1e6
#define NPARAM 10

// Control parameters storage object
class Param : public QObject
{
    Q_OBJECT

private:
    char RL[3];
    std::fstream paramFile;
    std::vector<std::unique_ptr<PVT>> cCurve;
    QVector<QVector<double>> cPar;

    bool load();

public:
    Param();
    ~Param();

    bool save();
    void setup();
    double get(const int ch, const int knob);
    bool set(const int ch, const int knob, const double val);

signals:
    void paramSend(const int ch, const int knob, const double val);
    void PVTSend(const int ch, const QVector<long> P, const QVector<long> V, const QVector<int> T);
};

#endif // PARAM_H
