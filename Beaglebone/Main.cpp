#include <signal.h>
#include <iostream>

#include <QCoreApplication>

#include "Param.h"
#include "Orthosis.h"

void SigHandler(int sig)
{
    if (sig == SIGINT)
        std::cout << "Caught SIGINT" << std::endl;

    if (sig == SIGTERM)
        std::cout << "Caught SIGTERM" << std::endl;

    QCoreApplication::quit();
}

int main(int argc, char* argv[])
{
    signal(SIGINT,  &SigHandler);
    signal(SIGTERM, &SigHandler);

    QCoreApplication app(argc, argv);

    std::unique_ptr<Orthosis> o;

    QStringList p;

    p.push_back("ttyO2");
    p.push_back("ttyO4");

    o.reset(new Orthosis(100, p));

    return app.exec();
}
