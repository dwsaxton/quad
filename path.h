#ifndef PATH_H
#define PATH_H

#include <Eigen/Geometry>
using namespace Eigen;

class Path {
public:
  // Target position at the given time
  virtual Vector3d position(double t) const = 0;
  // Target velocity at the given time
  virtual Vector3d velocity(double t) const = 0;
  // Time until intercept
  virtual double duration() const = 0;
  // Initial direction to accelerate in
  virtual Vector3d initialAccelerationDirection() const = 0;
};

#endif // PATH_H
