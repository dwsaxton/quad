#ifndef ONLINELEARNER_H
#define ONLINELEARNER_H

#include <Eigen/Geometry>
using namespace Eigen;

void TestOnlineLearner();

class OnlineLearner {
public:
  /**
   * Construct an online learner with the initial set of parameters and a given learning rate.
   */
  OnlineLearner(VectorXd const& param, double alpha = 0.1);

  VectorXd learnFromSample(std::function<double (VectorXd const&)> fn);

private:
  VectorXd param_;
  int n_;
  double alpha_; // learning rate
};

#endif // ONLINELEARNER_H
