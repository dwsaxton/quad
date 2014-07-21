#include "pathinterceptplanner.h"

#include <cmath>
#include <iostream>
using namespace std;

double newtonSearch(std::function< double (double)> fn, double x, bool* found) {
  // TODO make these parameters of this function?
  int maxIt = 40;
  double eps = 1e-6;
  double eps_deriv = 1e-5;
  for (int i = 0; i < maxIt; ++i) {
    double value = fn(x);
    if (abs(value) < eps) {
      *found = true;
      return x;
    }
    double derivative = (fn(x + eps_deriv) - value) / eps_deriv;
    double prev = x;
    x -= value / derivative;
    if (x < 0) {
      x = prev / 2.0 + 0.1;
    } else if (!isfinite(x) || x > 1e6) {
      x = 1e6;
    }
  }

  *found = false;
  return x;
}

double binarySearch(std::function<double (double)> fn, double left, double right, bool* found) {
  // TODO make these parameters of this function?
  int maxIt = 40;
  double eps = 1e-6;

  double valueLeft = fn(left);
  double valueRight = fn(right);

  if ((valueLeft > 0 && valueRight > 0) || (valueLeft < 0 && valueRight < 0)) {
    *found = false;
    return 0;
  }

  for (int i = 0; i < maxIt; ++i) {
    double mid = (left + right) / 2;
    double valueMid = fn(mid);

    if (abs(valueMid) < eps) {
      *found = true;
      return mid;
    }

    if ((valueLeft > 0) == (valueMid > 0)) {
      left = mid;
      valueLeft = valueMid;
    } else {
      right = mid;
      valueRight = valueMid;
    }
  }

  *found = false;
  return (left + right) / 2;
}