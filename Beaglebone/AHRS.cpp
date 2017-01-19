#include <iostream>
#include <iomanip>
#include <fstream>

#include "AHRS.h"

// Initialize static variables
double AHRS::t = 0.0;

// AHRS constructor
AHRS::AHRS(QString _port, int id_in):
    port(_port),
    id(id_in),
    qout(10),
    data(NFLOATS + 1),
    qs(nullptr),
    running(false),
    XOR(0)
{
    qs.reset(new QSerialPort(this));

    qs->setPortName(port);
    qs->setBaudRate(QSerialPort::Baud57600);

    msg << "Connecting AHRS " << id << " at port " << port.toStdString() << std::endl;
    printMsg();

    if (qs->open(QSerialPort::ReadWrite))
    {
        if (qs->bytesAvailable() < 100)
        {
            qs->write(QByteArray("#o1", 3));
            qs->waitForBytesWritten(-1);
            qs->waitForReadyRead(1500);
        }

        clearBuffer();

        msg << "AHRS " << id << " ready" << std::endl;
        printMsg();
    }
    else
    {
        id = port.toInt();

        msg << "Error opening port " << port.toStdString() << std::endl;
        printMsg();
    }
}

// AHRS destructor
AHRS::~ AHRS()
{
    msg << "Destroying AHRS " << id << std::endl;
    printMsg();
}

// Output the contents of buffer "msg" and clear
void AHRS::printMsg()
{
    std::cout << msg.rdbuf() << std::flush;
    msg.clear();
}

// Stop data output and clear the buffer
void AHRS::clearBuffer()
{
    qs->write(QByteArray("#o0", 3));
    qs->waitForBytesWritten(-1);

    while (qs->bytesAvailable() > 0)
    {
        qs->read(u.buffer, BUFSIZE);
        qs->waitForReadyRead(5);
    }
}

// Shift the input buffer and read a new byte
void AHRS::circshift(size_t size)
{
    if (!qs->bytesAvailable())
        qs->waitForReadyRead(-1);

    for (unsigned i = 0; i < size - 1; i++)
        u.buffer[i] = u.buffer[i + 1];

    qs->read(&u.buffer[size - 1], 1);
}

// Update current time from main thread
void AHRS::timeUpdate(const double t_main)
{
    t = t_main;
}

// Reset the AHRS, clear buffer and restart output
void AHRS::sync()
{
    msg << "Synchronizing AHRS " << id << std::endl;
    printMsg();

    clearBuffer();

    connect(qs.get(), &QSerialPort::readyRead, this, &AHRS::read);

    qs->write(QByteArray("#oab#o1", 7));
    qs->waitForBytesWritten(-1);

    running = true;

    msg << "AHRS " << id << " synchronized " << std::endl;
    printMsg();

    emit ready();
}

// Read a data block from the AHRS
void AHRS::read()
{
    if (qs->bytesAvailable() >= BUFSIZE)
    {
        XOR = 0;

        qs->read(u.buffer, BUFSIZE);

        for (int i = 1; i < BUFSIZE - 1; i++)
            XOR ^= u.buffer[i];

        while (u.header != 255 || u.checksum != XOR)
        {
            XOR ^= u.checksum;
            circshift(BUFSIZE);
            XOR ^= ~u.header;
        }

        data[0].push_back(t);
        for (unsigned i = 1; i < data.size(); i++)
        {
            data[i].push_back(u.q[i - 1]);
            qout[i - 1] = u.q[i - 1];
        }

        emit sendData(id, qout);
    }
}

// Write data buffer contents to disk
void AHRS::dump(const std::string pathDate)
{
    std::string file = pathDate + "-Rzr" + std::to_string(id) + ".txt";
    std::ofstream outFile(file);

    if (outFile.is_open())
    {
        msg << "Writing AHRS " << id << " data to " << file << std::endl;
        printMsg();

        outFile << std::setprecision(8) << std::fixed;
        for (uint i = 0; i < data[0].size(); i++)
        {
            for (uint ch = 0; ch < data.size(); ch++)
                outFile << std::setw(16) << data[ch][i];

            outFile << std::endl;
        }
        outFile.close();
    }

    for (uint i = 0; i < data.size(); i++)
        data[i].resize(0);
}

// Stop the AHRS and dump buffer data to disk
void AHRS::stop()
{
    if (running)
    {
        msg << "Stopping AHRS " << id << std::endl;
        printMsg();

        disconnect(qs.get(), &QSerialPort::readyRead, 0, 0);

        clearBuffer();
    }
}
