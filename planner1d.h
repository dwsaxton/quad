#ifndef PLANNER1D_H
#define PLANNER1D_H

/**
 * Test the various 1d planner classes.
 */
void TestPlanner1ds();

class Planner1d {
public:
  /**
   * @return the minimum duration it will take to get to the given target, under
   * the given maximum acceleration
   */
  virtual double getMinDuration(double max_accel) const = 0;
  /**
   * Set-up the planner so that we arrive at the target after @param duration
   * seconds have passed.
   */
  virtual void setupForDuration(double duration) = 0;
  /**
   * Set-up the planner so that we arrive at the target as soon as possible,
   * given the maximum duration.
   */
  virtual void setupForMaxAccel(double max_accel) = 0;
  /**
   * @return whether the planner is valid under the conditions imposed.
   */
  virtual bool isValid() const = 0;
  /**
   * @return the position and velocity at the given time.
   */
  virtual void getPosVel(double time, double *x, double *v) const = 0;
  /**
   * Set the target position and velocity.
   */
  virtual void setTarget(double x, double v) = 0;
  /**
   * @return the duration of the path, which will have been calculated after
   * one of the setup functions was called.
   */
  virtual double duration() const = 0;
};

#endif // PLANNER1D_H
