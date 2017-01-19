#ifndef ORTHOSIS_H
#define ORTHOSIS_H

#include <QTimer>
#include <QThread>
#include <QNetworkInterface>
#include <QElapsedTimer>
#include <QHostAddress>
#include <QUdpSocket>

#include "AHRS.h"
#include "Param.h"
#include "Control.h"

#define NOUT 8      // Size of output array for plotting

#define LPORT 8888  // UDP local listening port
#define PPORT 8889  // UDP plot destination port

#define MTRSR  25.0 // Motor angle read frequency
#define PLTSR  25.0 // Plot output frequency
#define PLTSA  25.0 // Plot output frequency (Android)

class Orthosis : public QObject
{
    Q_OBJECT

private:
    unsigned int sampRate;    // System sample rate
    QStringList SerialPorts;  // Serial ports for AHRS

    double t;                 // Time
    QTimer timer;             // Main loop timer
    QElapsedTimer etimer;     // Elapsed timer
    QUdpSocket socket;        // UDP server socket
    QHostAddress UDPClient;   // UDP client address
    QByteArray message;       // UDP message container
    double out[NOUT];         // Plot output vector
    quint16 pltPort;          // Plot socket port
    quint16 cmdPort;          // Command socket port
    QVector<float> q1, q2;    // AHRS output vectors
    QVector<double> mPos;     // Motor angles
    unsigned int readyIMUs;   // Synchronized AHRS counter
    unsigned int readyMotors; // Enabled motors counter

    // System status (0: disabled; 1: enabled; 3: running)
    char status;

    // Frame-counting variables (main loop, plot, motor)
    unsigned long cf, pf, mf;
    unsigned int pskip, mskip;

    // Thigh angles and vertical accelerations
    double rPitch, lPitch, rAcc, lAcc;

    // Pointers to AHRS objects
    std::unique_ptr<AHRS> Rzr1;
    std::unique_ptr<AHRS> Rzr2;

    // Pointers to motor objects
    std::shared_ptr<maxonMotor> Mtr1;
    std::shared_ptr<maxonMotor> Mtr2;

    // Object to store control parameters
    Param controlParam;

    // Motor control objects
    motorControl rMotorControl;
    motorControl lMotorControl;

    // Seperate threads for AHRS and motors
    QThread thread1, thread2, thread3, thread4;

public:
    Orthosis(const int sr, const QStringList SerialPorts);
    ~Orthosis();

    void enable();
    void start();
    void stop();
    void shutdown();

public slots:
    void loop();
    void razorReady();
    void razorGet(const int id, const QVector<float> qin);
    void motorReady();
    void motorGet(const WORD id, const double mIn);
    void readPendingDatagrams();

signals:
    void timeUpdate(const double t);
    void razorSync();
    void razorRead();
    void razorDump(const std::string pathDate);
    void razorStop();
    void motorHome();
    void motorRead(const double t);
    void motorDump(const std::string pathDate);
    void motorStop();
};

#endif // ORTHOSIS_H
