#include <math.h>
#include <iostream>

#include "Control.h"

// motorControl constructor
motorControl::motorControl() : mcid(++nc), mode(STANCE) {}

// motorControl destructor
motorControl::~motorControl()
{
    nc--;
}

// Control state machine operator
void motorControl::operator()(const double ownAnk, const double oppAnk, const double ownAcc)
{
    // Check if conditions to start swing are met
    if (mode == STANCE)
    {
        if (i < T.size())
        {
            emit motorAdd(mcid, P[i], V[i], static_cast<BYTE>(T[i]));
            i++;
        }

        if (ownAnk >= tw && oppAnk <= tp && sFrm > nf && ownAcc < -mt)
        {
            timer.start();
            mode = SWING;
            sFrm = 0;
        }
    }

    // Count number of consecutive static frames
    if (std::abs(ownAcc) <= st)
        sFrm++;
    else
        sFrm = 0;

    // Initiate swing phase
    if (mode == SWING)
    {
        t = static_cast<double>(timer.elapsed()) / 1000.0;

        if (t >= it)
        {
            i = 0;
            mode = STANCE;

            emit motorRun(mcid);
        }
    }
}

// Associate motor object
void motorControl::setMotor(std::shared_ptr<maxonMotor> motor)
{
    motor_ = motor;
    connect(this, &motorControl::motorAdd, motor_.get(), &maxonMotor::addPVT);
    connect(this, &motorControl::motorRun, motor_.get(), &maxonMotor::runIPM);
}

// Reset control state machine
void motorControl::reset()
{
    mode = STANCE;
    sFrm = 0;
    i = 0;
}

// Get parameters from remote interface
void motorControl::paramGet(const int ch, const int par, const double val)
{
    if (ch + 1 == mcid)
    {
        switch (par)
        {
        case 0: kr = val; break;
        case 1: ks = val; break;
        case 2: kw = val; break;
        case 3: cd = val; break;
        case 4: it = val; break;
        case 5: st = val; break;
        case 6: mt = val; break;
        case 7: nf = val; break;
        case 8: tw = val; break;
        case 9: tp = val; break;
        default:
            std::cout << "Unknown parameter" << std::endl;
        }
        ft = it + cd;
    }
}

// Get generated PVT array
void motorControl::PVTGet(const int ch, const QVector<long> p, const QVector<long> v, const QVector<int> t)
{
    if (ch + 1 == mcid)
    {
        P = p;
        V = v;
        T = t;
    }
}
