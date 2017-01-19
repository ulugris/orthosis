#include <iostream>

#include "Orthosis.h"

int motorControl::nc = 0;

// Convert quaternion to pitch angle
inline double quat2ang(QVector<float> q)
{
    return 180.0 / PI*asin(2.0*(q[2]*q[3] + q[0]*q[1]));
}

// Orthosis constructor
Orthosis::Orthosis(int sr, QStringList SerialPorts):
    sampRate(sr),
    pltPort(PPORT),
    q1(NFLOATS),
    q2(NFLOATS),
    mPos(2),
    status(0)
{
    // This is required for passing arguments through Qt signals and slots
    qRegisterMetaType<QVector<float>>("QVector<float>");
    qRegisterMetaType<QVector<long>>("QVector<long>");
    qRegisterMetaType<QVector<int>>("QVector<int>");
    qRegisterMetaType<std::string>("std::string");
    qRegisterMetaType<double>("double");
    qRegisterMetaType<long>("long");
    qRegisterMetaType<WORD>("WORD");
    qRegisterMetaType<BYTE>("BYTE");

    // Bind socket to local port, broadcast address
    socket.bind(LPORT, QAbstractSocket::ShareAddress);
    QString addr = socket.localAddress().toString();
    std::cout << "Socket bound to " << addr.toStdString() << ":" << LPORT << std::endl;

    // Set 1 ms base timer
    timer.setInterval(1);
    timer.setTimerType(Qt::PreciseTimer);

    // Motor frame skipping
    mskip = int(sampRate / MTRSR + 0.5);

    // Initialize AHRS and move to their own threads
    Rzr1.reset(new AHRS(SerialPorts[0], 1));
    Rzr2.reset(new AHRS(SerialPorts[1], 2));
    Rzr1->moveToThread(&thread1);
    Rzr2->moveToThread(&thread2);

    // Initialize motors (right is reversed)
    Mtr1.reset(new maxonMotor(true,  5000));
    Mtr2.reset(new maxonMotor(false, 5000));
    Mtr1->moveToThread(&thread3);
    Mtr2->moveToThread(&thread4);

    // Initialize motor controls
    rMotorControl.setMotor(Mtr1);
    lMotorControl.setMotor(Mtr2);

    // Execute "readPendingDatagrams" when the UDP socket receives data
    connect(&socket, &QUdpSocket::readyRead, this, &Orthosis::readPendingDatagrams);

    // Execute "loop" whenever the base timer times out
    connect(&timer, &QTimer::timeout, this, &Orthosis::loop);

    // Write parameters to control objects
    connect(&controlParam, &Param::paramSend, &rMotorControl, &motorControl::paramGet);
    connect(&controlParam, &Param::paramSend, &lMotorControl, &motorControl::paramGet);
    connect(&controlParam, &Param::PVTSend, &rMotorControl, &motorControl::PVTGet);
    connect(&controlParam, &Param::PVTSend, &lMotorControl, &motorControl::PVTGet);

    // Connect Orthosis signals to AHRS slots
    connect(this, &Orthosis::timeUpdate, &AHRS::timeUpdate);
    connect(this, &Orthosis::razorSync, Rzr1.get(), &AHRS::sync);
    connect(this, &Orthosis::razorSync, Rzr2.get(), &AHRS::sync);
    connect(this, &Orthosis::razorDump, Rzr1.get(), &AHRS::dump);
    connect(this, &Orthosis::razorDump, Rzr2.get(), &AHRS::dump);
    connect(this, &Orthosis::razorStop, Rzr1.get(), &AHRS::stop);
    connect(this, &Orthosis::razorStop, Rzr2.get(), &AHRS::stop);

    // Connect AHRS signals to Orthosis slots
    connect(Rzr1.get(), &AHRS::ready, this, &Orthosis::razorReady);
    connect(Rzr2.get(), &AHRS::ready, this, &Orthosis::razorReady);
    connect(Rzr1.get(), &AHRS::sendData, this, &Orthosis::razorGet);
    connect(Rzr2.get(), &AHRS::sendData, this, &Orthosis::razorGet);

    // Connect Orthosis signals to motor slots
    connect(this, &Orthosis::motorHome, Mtr1.get(), &maxonMotor::home);
    connect(this, &Orthosis::motorHome, Mtr2.get(), &maxonMotor::home);
    connect(this, &Orthosis::motorRead, Mtr1.get(), &maxonMotor::read);
    connect(this, &Orthosis::motorRead, Mtr2.get(), &maxonMotor::read);
    connect(this, &Orthosis::motorDump, Mtr1.get(), &maxonMotor::dump);
    connect(this, &Orthosis::motorDump, Mtr2.get(), &maxonMotor::dump);
    connect(this, &Orthosis::motorStop, Mtr1.get(), &maxonMotor::stop);
    connect(this, &Orthosis::motorStop, Mtr2.get(), &maxonMotor::stop);

    // Connect motor signals to Orthosis slots
    connect(Mtr1.get(), &maxonMotor::ready, this, &Orthosis::motorReady);
    connect(Mtr2.get(), &maxonMotor::ready, this, &Orthosis::motorReady);
    connect(Mtr1.get(), &maxonMotor::sendData, this, &Orthosis::motorGet);
    connect(Mtr2.get(), &maxonMotor::sendData, this, &Orthosis::motorGet);

    // Send control parameters to control objects
    controlParam.setup();

    thread1.start();
    thread2.start();
    thread3.start();
    thread4.start();
}

// Orthosis destructor
Orthosis::~Orthosis()
{
    std::cout << "Destroying Orthosis" << std::endl;

    thread1.quit();
    thread2.quit();
    thread3.quit();
    thread4.quit();

    shutdown();
}

// Enable motors
void Orthosis::enable()
{
    readyMotors = 0;

    emit motorHome();
}

// Start control loop
void Orthosis::start()
{
    memset(&out, 0, NOUT*sizeof(double));
    cf = 0; pf = 0; mf = 0;

    readyIMUs = 0;

    emit razorSync();
    emit timeUpdate(0.0);
}

// Stop control loop and dump sensor data to files
void Orthosis::stop()
{
    if (timer.isActive()) {
        timer.stop();

        tm *timestr;
        time_t now;
        char the_date[50];

        now = time(NULL);
        timestr = localtime(&now);

        strftime(the_date, 50, "%Y-%m-%d-%H%M%S", timestr);

        emit razorStop();
        emit razorDump("log/" + std::string(the_date));
        emit motorDump("log/" + std::string(the_date));
    }
}

// Shut down motors
void Orthosis::shutdown()
{
    stop();
    emit motorStop();
    rMotorControl.reset();
    lMotorControl.reset();
}

// Control loop
void Orthosis::loop()
{
    // Update current frame (in base timer resolution)
    if (etimer.elapsed() >= 1000.0 * cf / sampRate)
    {
        // Current time in seconds
        t = static_cast<double>(cf++) / sampRate;
        emit timeUpdate(t);

        // Get current thigh angles and accelerations
        rPitch = quat2ang(q1);
        lPitch =-quat2ang(q2);
        rAcc = q1[4] + cos(rPitch * PI/180);
        lAcc = q2[4] + cos(lPitch * PI/180);

        // Send motors to corresponding positions
        rMotorControl(rPitch, lPitch, rAcc);
        lMotorControl(lPitch, rPitch, lAcc);

        // Store output values
        out[0] = t;
        out[1] = rPitch + 35.0;
        out[2] = lPitch + 35.0;
        out[3] = mPos[0];
        out[4] = mPos[1];
        out[5] = 0;
        out[6] = 10*rAcc + 35.0;
        out[7] = 10*lAcc + 35.0;
    }

    if (cf >= pf) {
        socket.writeDatagram(QByteArray((const char *)out, NOUT*sizeof(double)), UDPClient, pltPort);
        pf += pskip;
    }

    if (cf >= mf) {
        emit motorRead(t);
        mf += mskip;
    }
}

// An AHRS is synchronized
void Orthosis::razorReady()
{
    if (++readyIMUs == 2)
    {
        timer.start();
        etimer.start();
    }
}

// Get AHRS data
void Orthosis::razorGet(const int id, const QVector<float> qin)
{
    if (id == 1) q1 = qin;
    if (id == 2) q2 = qin;
}

// A motor is enabled and at home position
void Orthosis::motorReady()
{
    if (++readyMotors == 2)
        status = 1;
}

// Get motor position
void Orthosis::motorGet(const WORD id, const double mIn)
{
    mPos[id-1] = mIn;
}

// UDP server command parser
void Orthosis::readPendingDatagrams()
{
    while (socket.hasPendingDatagrams())
    {
        message.resize(socket.pendingDatagramSize());
        socket.readDatagram(message.data(), message.size(), &UDPClient, &cmdPort);

        if (message == QString("Connect") || message == QString("Android"))
        {
            socket.writeDatagram(QByteArray("Ok"), UDPClient, cmdPort);
            socket.writeDatagram(&status, sizeof(status), UDPClient, cmdPort);

            double exportParam[2*NPARAM];
            for (int i = 0; i < NPARAM; i++)
            {
                exportParam[i] = controlParam.get(0, i);
                exportParam[i + NPARAM] = controlParam.get(1, i);
            }

            QByteArray paramList((const char *)exportParam, sizeof(exportParam));
            socket.writeDatagram(paramList, UDPClient, cmdPort);

            std::string IP = UDPClient.toString().toStdString().substr(7);
            std::cout << "Connected to " << IP << std::endl;

            // Sensor plot frame skipping
            if (message == QString("Connect"))
                pskip = int(sampRate / PLTSR + 0.5);
            else
                pskip = int(sampRate / PLTSA + 0.5);

            std::cout << "Setting plot rate to " << sampRate/pskip << " Hz" << std::endl;
        }
        else if (message == QString("On"))
        {
            try
            {
                if (!status)
                {
                    enable();
                    socket.writeDatagram(QByteArray("Ok"), UDPClient, cmdPort);
                }
            }
            catch (const char *e)
            {
                std::cout << "Exception in " << e << ", closing" << std::endl;
                return;
            }
        }
        else if (message == QString("Off"))
        {
            if (status > 0)
            {
                status = 0;
                controlParam.save();
                shutdown();
                socket.writeDatagram(QByteArray("Ok"), UDPClient, cmdPort);
            }
        }
        else if (message == QString("Start"))
        {
            if (status == 1)
            {
                status = 3;
                start();
                socket.writeDatagram(QByteArray("Ok"), UDPClient, cmdPort);
            }
        }
        else if (message == QString("Stop"))
        {
            if (status == 3)
            {
                status = 1;
                stop();
                socket.writeDatagram(QByteArray("Ok"), UDPClient, cmdPort);
            }
        }
        else if (message.size() == 24)
        {
            double *cmd = (double*)message.data();

            if (controlParam.set((int)cmd[0], (int)cmd[1], cmd[2]))
            {
                socket.writeDatagram(QByteArray("Ok"), UDPClient, cmdPort);
            }
            else
            {
                socket.writeDatagram(QByteArray("Err"), UDPClient, cmdPort);
            }
        }
        else
        {
            std::cout << "Unknown command \"" << message.data() << "\"" << std::endl;
        }
    }
}
