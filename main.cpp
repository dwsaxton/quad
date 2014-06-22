#include <QApplication>

#include "interface.h"
#include "mpc.h"
#include "qpsolver.h"

int main(int argc, char *argv[])
{
//    mpcTest();
    QP::test();
//     exit(0);
   
   QApplication app(argc, argv);
   Interface interface;
   interface.show();
   return app.exec();
}
