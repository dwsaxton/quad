#include "interceptplot.h"

#include <QPainter>

InterceptPlot::InterceptPlot(QWidget *parent) {
  t_min = 0;
  t_max = 10;
  f_min = -2;
  f_max = 2;
  width_ = 0;
  height_ = 0;
  steps_ = 100;
  intercept_time_ = 0;
}

void InterceptPlot::paintEvent(QPaintEvent *) {
  width_ = width();
  height_ = height();

  QPainter painter(this);

  // Axis
  painter.drawLine(coordToPoint(t_min, 0), coordToPoint(t_max, 0));

  for (int mono = 0; mono < 2; ++mono) {
    painter.setPen(QPen(mono == 0 ? Qt::red : Qt::blue, 1));
    QPointF prev;
  //   painter.setRenderHint(QPainter::Antialiasing);
    for (int i=1; i<steps_; ++i) {
      double t = t_min + i * (t_max - t_min) / steps_;
      QPointF next = coordToPoint(t, intercept_.f(t, mono));
      if (i > 0 ) {
        painter.drawLine(prev, next);
      }
      prev = next;
    }
  }

  painter.setPen(QPen(Qt::black, 4));
  painter.drawPoint(coordToPoint(intercept_time_, 0));
}

void InterceptPlot::setIntercept(LinearPlanner3d const& intercept, double intercept_time) {
  intercept_time_ = intercept_time;
  intercept_ = intercept;
  repaint();
}

QPointF InterceptPlot::coordToPoint(double x, double y) const {
  double frac_x = (x - t_min) / (t_max - t_min);
  double frac_y = (y - f_min) / (f_max - f_min);
  return QPointF(width_ * frac_x, height_ * (1 - frac_y));
}