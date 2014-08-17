#ifndef PROPELLERS_H
#define PROPELLERS_H

#include <Eigen/Geometry>
using namespace Eigen;

class Propellers {
public:
  Propellers();

  /**
   * Sets the input to the ith propeller. input is between 0 and 1, representing
   * the proportion of thrust the propeller will generate (more precisely,
   * this will set the speed of the propeller to be proportional to the square
   * root of input).
   */
  void setInput(int i, double input);
  /**
   * Similar as above, but set all inputs simultaneously.
   */
  void setInput(Vector4d const& input);

  /**
   * @return the current input to the propellers
   */
  Vector4d input() const { return input_; }

private:
  Vector4d input_; // the last set input
};

#endif // PROPELLERS_H
