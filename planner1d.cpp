#include "planner1d.h"

#include "linearplanner.h"
#include "linearplannermono.h"

#include <cassert>
#include <cmath>
using namespace std;

void TestFinalValue(Planner1d *planner, double x, double v, double duration, bool requireAlwaysValid, bool allowInvalidDuration = false) {
  planner->setTarget(x, v);
  planner->setupForDuration(duration);
  assert(!requireAlwaysValid || planner->isValid());
  if (!planner->isValid()) {
    return;
  }
  assert(allowInvalidDuration || abs(planner->duration() - duration) < 1e-5);
  double px, pv;
  planner->getPosVel(duration, &px, &pv);
  assert(abs(px - x) < 1e-5);
  assert(abs(pv - v) < 1e-5);
}

void Test(Planner1d *planner, bool requireAlwaysValid) {
  TestFinalValue(planner, 1, 4, 2, requireAlwaysValid);
  TestFinalValue(planner, -1, 3, 1, requireAlwaysValid);
  TestFinalValue(planner, 2, -7, 9, requireAlwaysValid);
  TestFinalValue(planner, -5, -8, 5, requireAlwaysValid);
  TestFinalValue(planner, 0, 0, 3, requireAlwaysValid, true);
  TestFinalValue(planner, 1, 0, 2, requireAlwaysValid);
  TestFinalValue(planner, -1, 0, 1, requireAlwaysValid);
  TestFinalValue(planner, 0, -7, 9, requireAlwaysValid);
  TestFinalValue(planner, 0, -8, 5, requireAlwaysValid);
  TestFinalValue(planner, -16.443, -5.9624, 5.5206, requireAlwaysValid); // a previously failing value
}

void TestValid(Planner1d *planner, double x, double v, double duration, bool expectedValidity) {
  planner->setTarget(x, v);
  planner->setupForDuration(duration);
  assert(planner->isValid() == expectedValidity);
}

void TestPlanner1ds() {
  LinearPlannerMono mono;
  LinearPlanner full;
  Test(&mono, false);
  Test(&full, true);

  // Test whether mono planner is valid or invalid under certain conditions
  TestValid(&mono, 1, 4, 2, true);
  TestValid(&mono, -1, 3, 1, false);
  TestValid(&mono, 2, -7, 9, false);
  TestValid(&mono, -5, -8, 5, false);
  TestValid(&mono, 0, 0, 5, true);
  TestValid(&mono, 0, 1, 5, false);
  TestValid(&mono, 1, 0, 5, false);
}
