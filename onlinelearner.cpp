#include "onlinelearner.h"

#include <cassert>
#include <iostream>
#include <random>
using namespace std;

OnlineLearner::OnlineLearner(VectorXd const& param, double alpha) {
  param_ = param;
  alpha_ = alpha;
  n_ = param.size();
  for (int i = 0; i < n_; ++i) {
    assert(param_[i] > 0); // OnlineLearner assumes the parameters are all strictly positive.
  }
}

VectorXd OnlineLearner::learnFromSample(std::function<double (VectorXd const&)> fn) {
  double eps = 0.01;
  double f0 = fn(param_);
  VectorXd df(n_);
  for (int i = 0; i < n_; ++i) {
    double di = param_[i] * eps;
    param_[i] += di;
    double fi = fn(param_);
    param_[i] -= di;
    df[i] = (fi - f0) / di;
  }

  // Now do steepest descent
  for (int i = 0; i < n_; ++i) {
    param_[i] -= alpha_ * df[i];
  }

  return param_;
}

void TestOnlineLearner() {
  default_random_engine generator;
  uniform_real_distribution<double> dist(-1.0, 1.0);
  uniform_real_distribution<double> dist_pos(0.1, 1.0);
  double a = dist_pos(generator);
  double b = dist_pos(generator);
  double c = dist_pos(generator);
  double d = dist_pos(generator);
//   cout << "a = " << a << " b = " << b << " c = " << c << " d = " << d << endl;
  
  VectorXd param(4);
  param << 1, 1, 1, 1;
  double alpha = 0.1;
  OnlineLearner learner(param, alpha);
  int steps = 1000;
  for (int i = 0; i < steps; ++i) {
    double x = dist(generator);
    double y = dist(generator);
    double z = dist(generator);
    auto function = [&] (VectorXd const& param) { return pow((param[0] - a) * x * x + (param[1] - b) * x * y + (param[2] - c) * y * y + (param[3] - d) * x * y * z, 2); };
    param = learner.learnFromSample(function);
//     if (i % 200 == 0) {
//       cout << "param = " << param.transpose() << endl;
//     }
  }

  double error = sqrt(pow(param[0] - a, 2) + pow(param[1] - b, 2) + pow(param[2] - c, 2) + pow(param[3] - d, 2));
  assert(error < 0.1);
}
