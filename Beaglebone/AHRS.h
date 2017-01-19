#ifndef AHRS_H
#define AHRS_H

#include <sstream>
#include <memory>

#include <QSerialPort>
#include <QVector>

#define NFLOATS 10
#define BUFSIZE (4*NFLOATS+2)

// Sparkfun Razor 9-DOF IMU class
class AHRS : public QObject
{
    Q_OBJECT

private:
    QString port;
    int id;

    static double t;

    QVector<float> qout;
    std::vector<std::vector<float> > data;
    std::unique_ptr<QSerialPort> qs;
    std::stringstream msg;
    bool running;
    char XOR;

#pragma pack(push, 1)
    union
    {
        char buffer[BUFSIZE];
        struct
        {
            uchar header;
            float q[NFLOATS];
            char checksum;
        };
    } u;
#pragma pack(pop)

    void printMsg();
    void clearBuffer();
    void circshift(size_t size);

public:
    AHRS(QString _port, int id_in);
    ~AHRS();

signals:
    void ready();
    void sendData(const int id, const QVector<float> qout);

public slots:
    static void timeUpdate(const double t_main);
    void sync();
    void read();
    void dump(const std::string pathDate);
    void stop();
};

#endif // AHRS_H
