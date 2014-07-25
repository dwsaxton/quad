#ifndef HIGHERCONTROLLER_H
#define HIGHERCONTROLLER_H

#include <Eigen/Geometry>
using namespace Eigen;
#include <QObject>

class Controller;
class LinearPlanner3d;
class Path;
class Quadratic3d;
class QuadState;
class SimpleQuadraticIntercept;

class HigherController : public QObject {
   Q_OBJECT
   
public:
  HigherController();
    
  bool haveControl() const { return have_control_; }
  void step();
      
public Q_SLOTS:
  void giveControl(bool haveControl);

private:
  Quadratic3d stateTargetToQuadratic() const;
  Path* interceptForTarget(const QuadState& state, LinearPlanner3d* simpleIntercept) const;

  Vector3d target_pos_;
  bool have_control_;
  Controller *controller_;
  double prev_intercept_duration_;
};

#endif // HIGHERCONTROLLER_H
