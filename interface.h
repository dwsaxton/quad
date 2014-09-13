#ifndef INTERFACE_H
#define INTERFACE_H

#include <QWidget>

#include "ui_interface.h"

class Interface : public QWidget, public Ui::Interface
{
Q_OBJECT

public:
  Interface();
  ~Interface();
  
private Q_SLOTS:
  void updateLabels();

  void onRunPauseSimulationClicked();
  void setControlAutomatic(bool automatic);
  void onAutoGoClicked();
  void onPropSlidersAdjusted();
  void onResetClicked();
  
private:
  double propInput(int i) const;
  QSlider *m_propInputs[4];
  bool control_automatic_;
};

#endif
