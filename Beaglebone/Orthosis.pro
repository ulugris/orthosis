QT += core
QT += serialport
QT += network
QT -= gui

TARGET = Orthosis
CONFIG += console
CONFIG += c++11
CONFIG -= app_bundle

TEMPLATE = app

SOURCES +=        \
    Main.cpp      \
    Orthosis.cpp  \
    AHRS.cpp      \
    EPOS2.cpp     \
    Control.cpp   \
    Param.cpp     \
    PVT.cpp

HEADERS +=        \
    MotorConfig.h \
    Orthosis.h    \
    AHRS.h        \
    EPOS2.h       \
    Control.h     \
    Param.h       \
    PVT.h

LIBS += -lEposCmd

target.path = /home/debian
INSTALLS += target
