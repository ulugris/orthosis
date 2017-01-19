#ifndef EPOS2_H
#define EPOS2_H

#include <string>
#include <vector>
#include <sstream>
#include <QObject>
#include <QVector>

typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned int DWORD;
typedef void* HANDLE;
typedef int BOOL;

#include "Definitions.h"

// Maxon EPOS2 motor class
class maxonMotor : public QObject {
    Q_OBJECT

private:
    BOOL reverse;
    long hoffset;

    static const double QC_PER_DEG;
    static HANDLE keyHandle;
    static WORD nMotors;

    std::vector<std::vector<double> > data;
    std::stringstream msg;
    DWORD errid;
    WORD motor;
    int qcs;

    void printMsg();
    void errChk(BOOL success, BOOL fatal = true);

public:
    maxonMotor(const bool rev, const long offset = 0);
    ~maxonMotor();

signals:
    void ready();
    void sendData(const WORD id, const double out);

public slots:
    void home();
    void read(double t);
    void addPVT(const WORD id, const long pos, const long vel, const BYTE dt);
    void runIPM(const WORD id);
    void dump(const std::string pathDate);
    void stop();
};

#endif // EPOS2_H
