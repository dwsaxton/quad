#ifndef QUADRATIC3D_H
#define QUADRATIC3D_H

#include <Eigen/Geometry>
using namespace Eigen;

void test_Quadratic3d();

/**
 * Quadratic of the form f(x) = a x^2 + b x + c, where a, b, c are 3d vectors
 */
class Quadratic3d {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Quadratic3d();
  Quadratic3d(Vector3d const& a, Vector3d const& b, Vector3d const& c);

  Vector3d a;
  Vector3d b;
  Vector3d c;

  // Calculates the length from 0 to x along the curve
  double length(double x) const;

  Vector3d eval(double x) const;

  // returns the "quadratic" 2 a x + b.
  Quadratic3d derivative() const;
};

#endif // QUADRATIC3D_H
