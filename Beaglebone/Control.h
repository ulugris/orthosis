#ifndef CONTROL_H
#define CONTROL_H

#include <memory>
#include <QObject>
#include <QElapsedTimer>

#include "PVT.h"
#include "EPOS2.h"

// Functor that controls a motor depending on AHRS inputs
class motorControl : public QObject {
    Q_OBJECT

private:
    static int nc;

    enum mode_t{STANCE, SWING};

    int mcid;
    mode_t mode;
    QElapsedTimer timer;
    QVector<long> P, V;
    QVector<int> T;
    std::shared_ptr<maxonMotor> motor_;
    double initOppAnk, initOwnAnk, pAcc, vAcc;
    double kr, ks, kw, cd, it, ft;
    double tw, tp, st, mt, t;
    int nf, i = 0, sFrm = 0;

public:
    motorControl();
    ~motorControl();

    void operator()(const double ownAnk, const double oppAnk, const double ownAcc);
    void setMotor(std::shared_ptr<maxonMotor> motor);
    void reset();

signals:
    void motorAdd(const WORD id, const long pos, const long vel, const BYTE dt);
    void motorRun(const WORD id);

public slots:
    void paramGet(const int ch, const int par, const double val);
    void PVTGet(const int ch, const QVector<long> p, const QVector<long> v, const QVector<int> t);
};

#endif // CONTROL_H
