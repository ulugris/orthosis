#include <iostream>
#include <iomanip>

#include "Param.h"

// Param constructor
Param::Param() : cCurve(2), cPar(2)
{
    RL[0] = 'R';
    RL[1] = 'L';
    cCurve[0].reset(new PVT(6, 160, 4096)); // R PVT array
    cCurve[1].reset(new PVT(6, 160, 4096)); // L PVT array
}

// Param destructor
Param::~Param()
{
    save();
}

// Load settings from file
bool Param::load()
{
    paramFile.open("Control.ini", std::fstream::in);

    if (paramFile.is_open())
    {
        std::cout << "Loading configuration file" << std::endl;
        double rval, lval;

        for (int i = 0; i < NPARAM; i++)
        {
            paramFile >> rval >> lval;
            cPar[0].append(rval);
            cPar[1].append(lval);
        }

        paramFile.close();

        return true;
    }

    return false;
}

// Write settings to file
bool Param::save()
{
    paramFile.open("Control.ini", std::fstream::out);

    if (paramFile.is_open())
    {
        std::cout << "Writing configuration file" << std::endl;
        paramFile << std::setprecision(3) << std::fixed;

        for (int i = 0; i < NPARAM; i++)
        {
            paramFile << std::setw(9) << cPar[0][i];
            paramFile << std::setw(9) << cPar[1][i];
            paramFile << std::endl;
        }

        paramFile.close();

        return true;
    }

    return false;
}

// Initialize parameters storage object
void Param::setup()
{
    // Load settings file
    bool fileFound = load();

    for (int i = 0; i < 2; i++)
    {
        if (fileFound == false) {
            cPar[i].append(40.0); // kr: maximum knee flexion (degrees)
            cPar[i].append(0.16); // ks: peak displacement parameter (non-dimensional)
            cPar[i].append(-0.1); // kw: peak width parameter (non-dimensional)
            cPar[i].append(0.70); // cl: cycle length (s)
            cPar[i].append(0.00); // it: delay from heel-off to cycle (seconds)
            cPar[i].append(0.05); // st: acceleration threshold to consider static frame (g)
            cPar[i].append(0.25); // mt: negative acceleration threshold to start cycle (g)
            cPar[i].append(  25); // nf: minimum static frames to start cycle
            cPar[i].append(10.0); // tw: own ankle threshold for stance (degrees)
            cPar[i].append( 4.0); // tp: opposite ankle threshold for stance (degrees)
        }

        cCurve[i]->gen(cPar[i][3], cPar[i][1], cPar[i][2], cPar[i][0]);

        emit PVTSend(i, cCurve[i]->getP(), cCurve[i]->getV(), cCurve[i]->getT());

        for (int par = 0; par < cPar[i].size(); par++)
            emit paramSend(i, par, cPar[i][par]);
    }
}

// Get parameters from remote interface
double Param::get(const int ch, const int knob)
{
    return cPar[ch][knob];
}

// Set parameters and send to control objects
bool Param::set(const int ch, const int knob, const double val)
{
    cPar[ch][knob] = val;

    std::cout << "Setting knob " << knob + 1 << " of "
              << RL[ch] << " channel to " << val << std::endl;

    emit paramSend(ch, knob, val);

    // Knobs 0-3 control PVT curve, feasibility must be checked before setting PVT array
    if (knob < 4)
    {
        cCurve[ch]->gen(cPar[ch][3], cPar[ch][1], cPar[ch][2], cPar[ch][0]);

        if (cCurve[ch]->check(MAXVEL, MAXACC))
        {
            std::cout << "  Generating PVT array for motor " << ch + 1 << ":" << std::endl;

            cCurve[ch]->disp();
            emit PVTSend(ch, cCurve[ch]->getP(), cCurve[ch]->getV(), cCurve[ch]->getT());
        }
        else
        {
            std::cout << "Unfeasible PVT array for motor " << ch + 1 << std::endl;

            return false;
        }
    }

    return true;
}
