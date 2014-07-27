#include <QApplication>

#include "interface.h"
#include "mpc.h"
#include "planner1d.h"
#include "qpsolver.h"
#include "quadratic3d.h"
#include "ssc.h"

int main(int argc, char *argv[])
{
//   mpcTest();
  QP::test();
  test_Quadratic3d();
  testSsc();
  TestPlanner1ds();

  QApplication app(argc, argv);
  Interface interface;
  interface.show();
  return app.exec();
}
