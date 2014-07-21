#ifndef INTERCEPTPLOT_H
#define INTERCEPTPLOT_H

#include <QWidget>

#include "simplequadraticintercept.h"

class InterceptPlot : public QWidget {
  Q_OBJECT

public:
  InterceptPlot(QWidget *parent);

  void setIntercept(SimpleQuadraticIntercept const& intercept, double intercept_time);
  
  double t_min;
  double t_max;
  double f_min;
  double f_max;

protected:
  void paintEvent(QPaintEvent *);
  QPointF coordToPoint(double x, double y) const;

private:
  SimpleQuadraticIntercept intercept_;
  int width_;
  int height_;
  int steps_;
  double intercept_time_;
};

#endif // INTERCEPTPLOT_H
