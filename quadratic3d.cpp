#include "quadratic3d.h"

#include <cmath>

// Integrates from 0 to x the expression sqrt{ u x^2 + v x + w }.
double quadraticSqrtIntegrate(double x, double u, double v, double w) {
  assert(u>=0); // haven't implemented u < 0

  if (u == 0) {
    return (2.0/3.0) * pow(v*x + w, 1.5) / v;
  }
  // else u > 0

  if (v == 0) {
    if (w > 0) {
      return 0.5*sqrt(u*x*x + w)*x + 0.5*w*asinh(u*x/sqrt(u*w))/sqrt(u);
    } else {
      return 0.5*sqrt(u*x*x + w)*x + 0.5*w*log(2*u*x + 2*sqrt(u*x*x + w)*sqrt(u))/sqrt(u);
    }
  }
  // else v != 0

  double discrim = 4*u*w-v*v;
  if (discrim > 0) {
    auto lambda = [&] (double y) -> double {
      return 0.5*sqrt(u*y*y + v*y + w)*y - 0.125*v*v*asinh((2*u*y + v)/sqrt(-v*v + 4*u*w))/pow(u,1.5) + 0.5*w*asinh((2*u*y + v)/sqrt(-v*v + 4*u*w))/sqrt(u) + 0.25*sqrt(u*y*y + v*y + w)*v/u;      
    };
    return lambda(x) - lambda(0);
  } else if (discrim == 0) {
    return 0.5*sqrt(u)*(x*x+v*x/u);
  } else /* discrim < 0 */ {
    return 0.5*sqrt(u*x*x + v*x + w)*x - 0.125*v*v*log(2*u*x + 2*sqrt(u*x*x + v*x + w)*sqrt(u) + v)/pow(u,1.5) + 0.5*w*log(2*u*x + 2*sqrt(u*x*x + v*x + w)*sqrt(u) + v)/sqrt(u) + 0.25*sqrt(u*x*x + v*x + w)*v/u;
  }
}

Quadratic3d::Quadratic3d(const Vector3d& a, const Vector3d& b, const Vector3d& c) {
  this->a = a;
  this->b = b;
  this->c = c;
}

Quadratic3d::Quadratic3d() {
  a.setZero();
  b.setZero();
  c.setZero();
}

double Quadratic3d::length(double x) const
{
  // We calculate the integral from 0 to x of the infinitesimal path element length, which is
  // given by taking the derivative of the path with respect to x (giving a 3-dimensional path
  // element), and integrating the l2 norm of this. Inside the square root of the integral, we
  // have an expression of the form || 2 a x + b ||^2 = u x^2 + v x + w, where:

  double u = 0;
  double v = 0;
  double w = 0;
  for (int i = 0; i < 3; ++i) {
    u += 4 * a(i) * a(i);
    v += 4 * a(i) * b(i);
    w += b(i) * b(i);
  }

  return quadraticSqrtIntegrate(x, u, v, w);
}

Vector3d Quadratic3d::eval(double x) const {
  return x * x * a + x * b + c;
}

Quadratic3d Quadratic3d::derivative() const {
  return Quadratic3d(Vector3d::Zero(), 2 * a, b);
}

void assert_approx_eq(double value, double target, double eps) {
  assert(value >= target - eps);
  assert(value <= target + eps);
}

void test_Quadratic3d() {
  assert_approx_eq(quadraticSqrtIntegrate(10, 1, 0, 7), 58.8598, 0.01); 
  assert_approx_eq(quadraticSqrtIntegrate(10, 1, 5, 7), 75.5983, 0.01);
  assert_approx_eq(quadraticSqrtIntegrate(10, 2, 4, 2), 84.8528, 0.01);
  assert_approx_eq(quadraticSqrtIntegrate(10, 2, 5, 2), 87.4948, 0.01);
  assert_approx_eq(Quadratic3d(Vector3d(4, 0, -3), Vector3d(3, 15, -5), Vector3d(06, 17, 4)).length(10), 583.1264, 0.01);
}