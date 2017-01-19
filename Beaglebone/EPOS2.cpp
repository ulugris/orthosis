#include <iostream>
#include <iomanip>
#include <fstream>
#include <math.h>

#include "EPOS2.h"
#include "MotorConfig.h"

// Initialize static constants
const double maxonMotor::QC_PER_DEG = ENCR4X*GEARRT/360.0;

// Initialize static variables
HANDLE maxonMotor::keyHandle = 0;
WORD maxonMotor::nMotors = 0;

// maxonMotor constructor
maxonMotor::maxonMotor(const bool rev, const long offset) : reverse(rev), hoffset(offset), data(2)
{
    if (!keyHandle)
    {
        char dev[] = "EPOS2";
#if RS232 == 1
        char ptc[] = "MAXON_RS232";
        char ifc[] = "RS232";
        char prt[] = RS232PORT;
#else
        char ptc[] = "MAXON SERIAL V2";
        char ifc[] = "USB";
        char prt[] = "USB0";
#endif
        keyHandle = VCS_OpenDevice(dev, ptc, ifc, prt, &errid);
    }

    if (!keyHandle)
        errChk(false);

    // Increase motor counter if successful
    nMotors++;

    // Assign incremental motor number
    motor = nMotors;

    // Display current error codes
    BYTE nErr = 0;
    DWORD errCode = 0;
    if (VCS_GetNbOfDeviceError(keyHandle, motor, &nErr, &errid))
    {
        for (BYTE e = 1; e <= nErr; e++)
        {
            if (e == 1)
            {
                msg << "Clearing errors:" << std::endl;
                printMsg();
            }
            errChk(VCS_GetDeviceErrorCode(keyHandle, motor, e, &errCode, &errid), false);

            msg << " Error 0x" << std::hex << errCode << std::dec << std::endl;
            printMsg();
        }
    }

    // Reset motor state
    errChk(VCS_ClearFault(keyHandle, motor, &errid));
    errChk(VCS_SetDisableState(keyHandle, motor, &errid));

    // Set general motor parameters
    DWORD written;
    DWORD maxVel = MAXVEL;
    errChk(VCS_SetMotorType(keyHandle, motor, MT_EC_BLOCK_COMMUTATED_MOTOR, &errid));
    errChk(VCS_SetEcMotorParameter(keyHandle, motor, NOMCUR, MAXCUR, THCONS, NUMPOL, &errid));
    errChk(VCS_SetObject(keyHandle, motor, 0x6410, 0x04, &maxVel, sizeof maxVel, &written, &errid));
    errChk(VCS_SetMaxProfileVelocity(keyHandle, motor, MAXVEL, &errid));
    errChk(VCS_SetMaxAcceleration(keyHandle, motor, MAXACC, &errid));
    errChk(VCS_SetMaxFollowingError(keyHandle, motor, 180*QC_PER_DEG, &errid));

    // Set the reverse rotation bit if necessary
    DWORD sConfig;
    if (reverse)
        sConfig = 3;
    else
        sConfig = 0;

    errChk(VCS_SetSensorType(keyHandle, motor, ST_INC_ENCODER_2CHANNEL, &errid));
    errChk(VCS_SetObject(keyHandle, motor, 0x2210, 0x04, &sConfig, sizeof sConfig, &written, &errid));

    // Set PID controller parameters
    if (motor == 1)
    {
        errChk(VCS_SetCurrentRegulatorGain(keyHandle, motor, RCURPGAIN, RCURIGAIN, &errid));

        errChk(VCS_SetVelocityRegulatorGain(keyHandle, motor, RSPDPGAIN, RSPDIGAIN, &errid));
        errChk(VCS_SetVelocityRegulatorFeedForward(keyHandle, motor, RSPDFFVEL, RSPDFFACC, &errid));

        errChk(VCS_SetPositionRegulatorGain(keyHandle, motor, RPOSPGAIN, RPOSIGAIN, RPOSDGAIN, &errid));
        errChk(VCS_SetPositionRegulatorFeedForward(keyHandle, motor, RPOSFFVEL, RPOSFFACC, &errid));
    }
    else
    {
        errChk(VCS_SetCurrentRegulatorGain(keyHandle, motor, LCURPGAIN, LCURIGAIN, &errid));

        errChk(VCS_SetVelocityRegulatorGain(keyHandle, motor, LSPDPGAIN, LSPDIGAIN, &errid));
        errChk(VCS_SetVelocityRegulatorFeedForward(keyHandle, motor, LSPDFFVEL, LSPDFFACC, &errid));

        errChk(VCS_SetPositionRegulatorGain(keyHandle, motor, LPOSPGAIN, LPOSIGAIN, LPOSDGAIN, &errid));
        errChk(VCS_SetPositionRegulatorFeedForward(keyHandle, motor, LPOSFFVEL, LPOSFFACC, &errid));
    }
}

// maxonMotor destructor
maxonMotor::~maxonMotor()
{
    if (keyHandle)
    {
        errChk(VCS_SetDisableState(keyHandle, motor, &errid));
        nMotors--;

        if (nMotors == 0)
        {
            msg << "EPOS2 cleared" << std::endl;
            printMsg();

            keyHandle = 0;
        }
    }
}

// Output the contents of buffer "msg" and clear
void maxonMotor::printMsg()
{
    std::cout << msg.rdbuf() << std::flush;
    msg.clear();
}

// EPOS2 error parser
void maxonMotor::errChk(BOOL success, BOOL fatal)
{
    if (!success)
    {
        char errBuff[100];
        if (VCS_GetErrorInfo(errid, errBuff, 100))
        {
            msg << "EPOS2 error " << errid << " (" << errBuff << ")" << std::endl;
            printMsg();

            if (fatal)
                throw "EPOS2";
        }

    }
}

// Enable motor and find home position
void maxonMotor::home()
{
    msg << "Motor " << motor << " homing" << std::endl;
    printMsg();

    errChk(VCS_ClearFault(keyHandle, motor, &errid));

    // Find home positions at mechanical end stop
    errChk(VCS_SetOperationMode(keyHandle, motor, OMD_HOMING_MODE, &errid));
    errChk(VCS_SetHomingParameter(keyHandle, motor, HOMACC, HOMSPS, HOMSPI, hoffset, HOMCUR, 0, &errid));
    errChk(VCS_ActivateHomingMode(keyHandle, motor, &errid));
    errChk(VCS_SetEnableState(keyHandle, motor, &errid));
    errChk(VCS_FindHome(keyHandle, motor, HM_CURRENT_THRESHOLD_NEGATIVE_SPEED, &errid));
    errChk(VCS_WaitForHomingAttained(keyHandle, motor, HOMTMO, &errid));

    // Activate interpolated position mode
    errChk(VCS_SetOperationMode(keyHandle, motor, OMD_INTERPOLATED_POSITION_MODE, &errid));
    errChk(VCS_ActivateInterpolatedPositionMode(keyHandle, motor, &errid));
    errChk(VCS_ClearIpmBuffer(keyHandle, motor, &errid));

    msg << "Motor " << motor << " ready" << std::endl;
    printMsg();

    emit ready();
}

// Motor data storage
void maxonMotor::read(double t)
{
    // Get actual position
    errChk(VCS_GetPositionIs(keyHandle, motor, &qcs, &errid));
    double pos = static_cast<double>(qcs) / QC_PER_DEG;

    data[0].push_back(t);
    data[1].push_back(pos);

    emit sendData(motor, pos);
}

// Add PVT point to IPM buffer
void maxonMotor::addPVT(WORD m, long pos, long vel, BYTE dt)
{
    if (m == motor)
    {
        errChk(VCS_AddPvtValueToIpmBuffer(keyHandle, motor, pos, vel, dt, &errid));

        msg << "Sending PVT: ";
        msg << std::setw(5) << pos << std::setw(6) << vel << std::setw(5) << (int)dt;
        msg << " to motor " << motor;

        if (dt == 0)
        {
            DWORD freeBuff;
            errChk(VCS_GetFreeIpmBufferSize(keyHandle, motor, &freeBuff, &errid));
            msg << " (" << freeBuff << " free) " << std::endl;
        }
        else
            msg << std::endl;

        printMsg();
    }
}

// Motor go-to-position command
void maxonMotor::runIPM(WORD m)
{
    if (m == motor)
    {
        errChk(VCS_StartIpmTrajectory(keyHandle, motor, &errid));

        msg << "Starting IPM on motor " << motor << std::endl;
        printMsg();
    }
}

// Dump sensor data to file
void maxonMotor::dump(const std::string pathDate)
{
    std::string file = pathDate + "-Mtr" + std::to_string(motor) + ".txt";
    std::ofstream outFile(file);

    if (outFile.is_open())
    {
        msg << "Writing motor " << motor << " data to " << file << std::endl;
        printMsg();

        outFile << std::setprecision(8) << std::fixed;
        for (uint i = 0; i < data[0].size(); i++)
        {
            for (uint c = 0; c < data.size(); c++)
                outFile << std::setw(16) << data[c][i];

            outFile << std::endl;
        }
        outFile.close();
    }

    for (uint i = 0; i < data.size(); i++)
        data[i].resize(0);    
}

// Disable motor
void maxonMotor::stop()
{
    if (keyHandle)
    {
        errChk(VCS_SetDisableState(keyHandle, motor, &errid));

        msg << "Motor " << motor << " disabled" << std::endl;
        printMsg();
    }
}
