#include "planner1d.h"

#include "linearplanner.h"
#include "linearplannermono.h"

#include <cassert>
#include <cmath>
using namespace std;

void TestFinalValueWithDuration(Planner1d *planner, double x, double v, double duration, bool requireAlwaysValid, bool allowInvalidDuration = false) {
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

void TestFinalValueWithAcceleration(Planner1d *planner, double x, double v, double acceleration, bool requireAlwaysValid) {
  planner->setTarget(x, v);
  planner->setupForDuration(acceleration);
  assert(!requireAlwaysValid || planner->isValid());
  if (!planner->isValid()) {
    return;
  }
  double px, pv;
  planner->getPosVel(planner->duration(), &px, &pv);
  assert(abs(px - x) < 1e-5);
  assert(abs(pv - v) < 1e-5);
}

void Test(Planner1d *planner, bool requireAlwaysValid) {
  TestFinalValueWithDuration(planner, 1, 4, 2, requireAlwaysValid);
  TestFinalValueWithDuration(planner, -1, 3, 1, requireAlwaysValid);
  TestFinalValueWithDuration(planner, 2, -7, 9, requireAlwaysValid);
  TestFinalValueWithDuration(planner, -5, -8, 5, requireAlwaysValid);
  TestFinalValueWithDuration(planner, 0, 0, 3, requireAlwaysValid, true);
  TestFinalValueWithDuration(planner, 1, 0, 2, requireAlwaysValid);
  TestFinalValueWithDuration(planner, -1, 0, 1, requireAlwaysValid);
  TestFinalValueWithDuration(planner, 0, -7, 9, requireAlwaysValid);
  TestFinalValueWithDuration(planner, 0, -8, 5, requireAlwaysValid);
  TestFinalValueWithDuration(planner, -16.443, -5.9624, 5.5206, requireAlwaysValid); // a previously failing value
  TestFinalValueWithAcceleration(planner, 1, 4, 4, requireAlwaysValid);
  TestFinalValueWithAcceleration(planner, -1, 3, 10, requireAlwaysValid);
  TestFinalValueWithAcceleration(planner, 2, -7, 3, requireAlwaysValid);
  TestFinalValueWithAcceleration(planner, -5, -8, 7, requireAlwaysValid);
  TestFinalValueWithAcceleration(planner, 0, 0, 9, requireAlwaysValid);
  TestFinalValueWithAcceleration(planner, 1, 0, 6, requireAlwaysValid);
  TestFinalValueWithAcceleration(planner, -1, 0, 5, requireAlwaysValid);
  TestFinalValueWithAcceleration(planner, 0, -7, 20, requireAlwaysValid);
  TestFinalValueWithAcceleration(planner, 0, -8, 100, requireAlwaysValid);
}

void TestValidForDuration(Planner1d *planner, double x, double v, double duration, bool expectedValidity) {
  planner->setTarget(x, v);
  planner->setupForDuration(duration);
  assert(planner->isValid() == expectedValidity);
}

void TestValidForAcceleration(Planner1d *planner, double x, double v, double max_accel, bool expectedValidity) {
  planner->setTarget(x, v);
  planner->setupForMaxAccel(max_accel);
  assert(planner->isValid() == expectedValidity);
}

void TestPlanner1ds() {
  LinearPlannerMono mono;
  LinearPlanner full;
  Test(&mono, false);
  Test(&full, true);

  // Test whether mono planner is valid or invalid under certain conditions
  TestValidForDuration(&mono, 1, 4, 2, true);
  TestValidForDuration(&mono, -1, 3, 1, false);
  TestValidForDuration(&mono, 2, -7, 9, false);
  TestValidForDuration(&mono, -5, -8, 5, false);
  TestValidForDuration(&mono, 0, 0, 5, true);
  TestValidForDuration(&mono, 0, 1, 5, false);
  TestValidForDuration(&mono, 1, 0, 5, false);
  TestValidForAcceleration(&mono, 408978, 2558, 40, true); // a previously failing value
}
