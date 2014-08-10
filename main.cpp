#include <QApplication>

#include "interface.h"
#include "linearplanner3d.h"
#include "onlinelearner.h"
#include "pathinterceptplanner.h"
#include "planner1d.h"
#include "qpsolver.h"
#include "quadratic3d.h"
#include "rotationplanner.h"
#include "ssc.h"

int main(int argc, char *argv[])
{
  QP::test();
  test_Quadratic3d();
  testSsc();
  TestPlanner1ds();
  TestPathInterceptPlanners();
  TestLinearPlanner3d();
  TestRotationPlanner();
  TestOnlineLearner();

  QApplication app(argc, argv);
  Interface interface;
  interface.show();
  return app.exec();
}
