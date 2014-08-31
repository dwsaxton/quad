#include <QApplication>

#include "globals.h"
#include "interface.h"
#include "linearplanner3d.h"
#include "onlinelearner.h"
#include "pathinterceptplanner.h"
#include "planner1d.h"
#include "qpsolver.h"
#include "quadratic3d.h"
#include "rotationplanner.h"
#include "ssc.h"

#include "hw/uart.h"
#include <iostream>
using namespace std;

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

  Globals & globals = Globals::self();

  Uart *uart = globals.uart();
  
  if (globals.environment() == Globals::OnBoard) {
    while (true) {
      uart->writeMessage("Hello");
      globals.sleep_us(1000000);
    }
  } else {
    while (true) {
      string message = uart->getMessage();
      if (!message.empty()) {
        cout << message << endl;
      }
      globals.sleep_us(100000);
    }
  }

  if (globals.environment() == Globals::OnBoard) {
    QCoreApplication app(argc, argv);
    return app.exec();
  } else {
    QApplication app(argc, argv);
    Interface interface;
    interface.show();
    return app.exec();
  }
}
