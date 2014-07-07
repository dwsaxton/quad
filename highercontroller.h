#ifndef HIGHERCONTROLLER_H
#define HIGHERCONTROLLER_H

#include <Eigen/Geometry>
using namespace Eigen;
#include <QObject>

class Controller;
class Quadratic3d;
class QuadState;

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
  Quadratic3d interceptForTarget(const QuadState& state) const;

  Vector3d target_pos_;
  bool have_control_;
  Controller *controller_;
};

#endif // HIGHERCONTROLLER_H
