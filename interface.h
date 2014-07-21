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
  void propInputsChanged();
  void updateLabels();
  void resetQuad();
  void runPauseQuad();
  void setEnvironmentSimulation();
  void setEnvironmentActual();
  
private:
  double propInput( int i ) const;
  QSlider *m_propInputs[4];
};

#endif
